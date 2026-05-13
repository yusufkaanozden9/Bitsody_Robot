import sys
import argparse
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import config


def main():
    parser = argparse.ArgumentParser(
        description="G1 Motion Control - Playback or Walking",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python main.py play positions.json          # Play motion from JSON file
  python main.py play t1.json --delay 100     # Custom frame delay (ms)
  python main.py walk                         # Start bipedal walking
        """,
    )

    subparsers = parser.add_subparsers(dest="mode", help="Operation mode")

    # Playback mode
    play_parser = subparsers.add_parser("play", help="Playback motion from JSON file")
    play_parser.add_argument("filename", help="JSON file in positions/ directory")
    play_parser.add_argument(
        "--delay",
        type=int,
        default=100,
        metavar="MS",
        help="Delay between frames in milliseconds (default: 100)",
    )

    # Walking mode
    walk_parser = subparsers.add_parser("walk", help="Start bipedal walking")
    walk_parser.add_argument(
        "--period",
        type=float,
        default=1.2,
        metavar="S",
        help="Gait cycle period in seconds (default: 1.2s for stable walking)",
    )
    walk_parser.add_argument(
        "--stride",
        type=float,
        default=0.2,
        metavar="M",
        help="Stride length in meters (default: 0.2m)",
    )

    args = parser.parse_args()

    if not args.mode:
        parser.print_help()
        sys.exit(1)

    # Initialize DDS communication
    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)

    if args.mode == "play":
        # Import g1_player here to avoid unnecessary loading
        from g1_player import load_frames, Custom

        try:
            frames = load_frames(args.filename)
        except FileNotFoundError as e:
            print(f"ERROR: {e}")
            sys.exit(1)

        print(f"\n{'='*60}")
        print(f"  MOTION PLAYBACK")
        print(f"{'='*60}")
        print(f"  File   : {args.filename}")
        print(f"  Frames : {len(frames)}")
        print(f"  Delay  : {args.delay} ms")
        print(f"  Note   : First 5 seconds are warmup (no motion)")
        print(f"{'='*60}\n")

        custom = Custom(frames=frames, delay_ms=args.delay)
        custom.Init()
        custom.Start()

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n\nShutdown...")

    elif args.mode == "walk":
        # Import walk controller
        from walk import WalkController

        print(f"\n{'='*60}")
        print(f"  BIPEDAL WALKING")
        print(f"{'='*60}")
        print(f"  Gait Period : {args.period}s")
        print(f"  Stride Len  : {args.stride}m")
        print(f"  Expected v  : {args.stride / args.period:.2f} m/s")
        print(f"  Note        : First 5 seconds are warmup (no motion)")
        print(f"{'='*60}\n")

        walk_controller = WalkController(
            gait_period=args.period,
            stride_length=args.stride,
            warmup_dur=5.0,
        )
        walk_controller.Init()
        walk_controller.Start()

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n\nShutdown...")


if __name__ == "__main__":
    main()
