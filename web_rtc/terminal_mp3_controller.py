import argparse
import asyncio
import contextlib
import io
import json
import logging
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

VOLUME_MIN = 0
VOLUME_MAX = 10


class TerminalKeyReader:
    def __enter__(self):
        if os.name != "nt":
            import termios
            import tty

            self._termios = termios
            self._fd = sys.stdin.fileno()
            self._old_settings = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if os.name != "nt" and hasattr(self, "_old_settings"):
            self._termios.tcsetattr(
                self._fd,
                self._termios.TCSADRAIN,
                self._old_settings,
            )

    def read_key(self):
        if os.name == "nt":
            import msvcrt

            key = msvcrt.getwch()
            if key in ("\x00", "\xe0"):
                return msvcrt.getwch()
            return key
        return sys.stdin.read(1)


def parse_args():
    default_mp3 = Path(__file__).with_name("dora-doradura-mp3.mp3")
    parser = argparse.ArgumentParser(
        description="Go2でMP3を再生し、ターミナルから音量を操作します。",
    )
    parser.add_argument(
        "--mp3",
        default=str(default_mp3),
        help="アップロードして再生するMP3またはWAVファイル。",
    )
    parser.add_argument(
        "--method",
        choices=("localsta", "localap", "remote"),
        default="localsta",
        help="接続方式。",
    )
    parser.add_argument(
        "--ip",
        default=os.environ.get("UNITREE_GO2_IP"),
        help="localsta接続で使うGo2のIPアドレス。UNITREE_GO2_IPも使用できます。",
    )
    parser.add_argument("--serial", help="ロボットのシリアル番号。")
    parser.add_argument("--username", help="remote接続で使うUnitreeアカウントのメールアドレス。")
    parser.add_argument("--password", help="remote接続で使うUnitreeアカウントのパスワード。")
    parser.add_argument("--region", default="global", choices=("global", "cn"))
    parser.add_argument("--aes-key", help="必要な端末で使うAES-128キー。")
    return parser.parse_args()


def create_connection(args):
    from unitree_webrtc_connect.constants import WebRTCConnectionMethod
    from unitree_webrtc_connect.webrtc_driver import UnitreeWebRTCConnection

    method = {
        "localsta": WebRTCConnectionMethod.LocalSTA,
        "localap": WebRTCConnectionMethod.LocalAP,
        "remote": WebRTCConnectionMethod.Remote,
    }[args.method]

    if method == WebRTCConnectionMethod.LocalSTA:
        if not args.ip and not args.serial:
            raise ValueError("localsta接続には --ip、--serial、または UNITREE_GO2_IP が必要です。")
        return UnitreeWebRTCConnection(
            method,
            ip=args.ip,
            serialNumber=args.serial,
            aes_128_key=args.aes_key,
        )

    if method == WebRTCConnectionMethod.Remote:
        missing = [
            name
            for name in ("serial", "username", "password")
            if not getattr(args, name)
        ]
        if missing:
            raise ValueError(f"remote接続には次の引数が必要です: {', '.join('--' + item for item in missing)}")
        return UnitreeWebRTCConnection(
            method,
            serialNumber=args.serial,
            username=args.username,
            password=args.password,
            region=args.region,
            aes_128_key=args.aes_key,
            device_type="Go2",
        )

    return UnitreeWebRTCConnection(method, aes_128_key=args.aes_key)


def parse_response_data(response):
    data = response.get("data", {}).get("data", "{}")
    if isinstance(data, bytes):
        data = data.decode("utf-8")
    if isinstance(data, str):
        return json.loads(data or "{}")
    if isinstance(data, dict):
        return data
    return {}


async def find_audio_uuid(audio_hub, audio_name):
    response = await audio_hub.get_audio_list()
    audio_list = parse_response_data(response).get("audio_list", [])
    for item in audio_list:
        if item.get("CUSTOM_NAME") == audio_name:
            return item.get("UNIQUE_ID")
    return None


async def ensure_audio_uploaded(audio_hub, audio_path):
    audio_path = Path(audio_path).expanduser().resolve()
    if not audio_path.exists():
        raise FileNotFoundError(f"音声ファイルが見つかりません: {audio_path}")

    audio_name = audio_path.stem
    uuid = await find_audio_uuid(audio_hub, audio_name)
    if uuid:
        print(f"ロボット上に同名の音声があります: {audio_name}")
        return uuid

    print(f"音声をロボットへアップロードしています: {audio_path}")
    with contextlib.redirect_stdout(io.StringIO()):
        await audio_hub.upload_audio_file(str(audio_path))

    for _ in range(10):
        await asyncio.sleep(0.5)
        uuid = await find_audio_uuid(audio_hub, audio_name)
        if uuid:
            print(f"アップロードが完了しました: {audio_name}")
            return uuid

    raise RuntimeError("アップロードは完了しましたが、AudioHub内に音声ファイルが見つかりません。")


async def get_volume(conn):
    from unitree_webrtc_connect.constants import RTC_TOPIC

    response = await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["VUI"],
        {"api_id": 1004},
    )
    status = response.get("data", {}).get("header", {}).get("status", {})
    if status.get("code") != 0:
        raise RuntimeError(f"音量を取得できませんでした: {status}")
    return int(parse_response_data(response).get("volume", 0))


async def set_volume(conn, volume):
    from unitree_webrtc_connect.constants import RTC_TOPIC

    volume = max(VOLUME_MIN, min(VOLUME_MAX, int(volume)))
    await conn.datachannel.pub_sub.publish_request_new(
        RTC_TOPIC["VUI"],
        {
            "api_id": 1003,
            "parameter": {"volume": volume},
        },
    )
    return volume


async def read_key_async(reader):
    loop = asyncio.get_event_loop()
    return await loop.run_in_executor(None, reader.read_key)


def print_controls(volume):
    print("")
    print("操作方法")
    print("  p / Enter : 音声を再生")
    print("  + / =     : 音量を上げる")
    print("  - / _     : 音量を下げる")
    print("  0-9       : 音量を直接指定")
    print("  q         : 終了")
    print("")
    print(f"現在の音量: {volume}/{VOLUME_MAX}")


async def interactive_loop(conn, audio_hub, audio_uuid):
    current_volume = await get_volume(conn)
    print_controls(current_volume)

    with TerminalKeyReader() as reader:
        while True:
            key = await read_key_async(reader)

            if key in ("q", "Q", "\x03"):
                print("\n終了します。")
                break

            if key in ("p", "P", "\r", "\n"):
                await audio_hub.play_by_uuid(audio_uuid)
                print("音声を再生しています。")
                continue

            if key in ("+", "="):
                current_volume = await set_volume(conn, current_volume + 1)
                print(f"音量: {current_volume}/{VOLUME_MAX}")
                continue

            if key in ("-", "_"):
                current_volume = await set_volume(conn, current_volume - 1)
                print(f"音量: {current_volume}/{VOLUME_MAX}")
                continue

            if key.isdigit():
                current_volume = await set_volume(conn, int(key))
                print(f"音量: {current_volume}/{VOLUME_MAX}")


async def main():
    args = parse_args()

    from unitree_webrtc_connect.webrtc_audiohub import WebRTCAudioHub

    conn = create_connection(args)

    try:
        await conn.connect()
        audio_hub = WebRTCAudioHub(conn, logger)
        audio_uuid = await ensure_audio_uploaded(audio_hub, args.mp3)
        await interactive_loop(conn, audio_hub, audio_uuid)
    finally:
        await conn.disconnect()


def run_main():
    if hasattr(asyncio, "run"):
        return asyncio.run(main())

    loop = asyncio.get_event_loop()
    return loop.run_until_complete(main())


if __name__ == "__main__":
    try:
        run_main()
    except KeyboardInterrupt:
        print("\nユーザー操作により中断しました。")
    except Exception as exc:
        logger.error("エラーが発生しました: %s", exc)
        sys.exit(1)
