#!/usr/bin/env python3

import argparse
import json
import os
import socket
import sys


HOST = os.environ.get("PEPPY_ISAAC_COMMAND_HOST", "127.0.0.1")
PORT = int(os.environ.get("PEPPY_ISAAC_COMMAND_PORT", "5556"))


def send_command(command: dict) -> None:
    payload = (json.dumps(command) + "\n").encode("utf-8")

    with socket.create_connection((HOST, PORT), timeout=5.0) as sock:
        sock.sendall(payload)

    print(json.dumps(command, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Runtime commander for the OpenArm Isaac Sim node."
    )

    sub = parser.add_subparsers(dest="subcommand", required=True)

    # arm
    arm_parser = sub.add_parser("arm")
    arm_parser.add_argument("side", choices=["left", "right"])
    arm_parser.add_argument("positions", nargs=7, type=float)

    # release
    release_parser = sub.add_parser("release")
    release_parser.add_argument("side", choices=["left", "right"])

    # joints
    sub.add_parser("joints")

    # robot root translation
    robot_parser = sub.add_parser("robot")
    robot_parser.add_argument("x", type=float)
    robot_parser.add_argument("y", type=float)
    robot_parser.add_argument("z", type=float)

    # scene
    scene_parser = sub.add_parser("scene")
    scene_parser.add_argument(
        "scene_name",
        choices=["tabletop", "shelf_reach"],
    )

    # spawn
    spawn_parser = sub.add_parser("spawn")
    spawn_parser.add_argument("name")
    spawn_parser.add_argument("path")
    spawn_parser.add_argument("x", type=float)
    spawn_parser.add_argument("y", type=float)
    spawn_parser.add_argument("z", type=float)
    spawn_parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Uniform object scale.",
    )

    # move
    move_parser = sub.add_parser("move")
    move_parser.add_argument("name")
    move_parser.add_argument("x", type=float)
    move_parser.add_argument("y", type=float)
    move_parser.add_argument("z", type=float)

    # remove
    remove_parser = sub.add_parser("remove")
    remove_parser.add_argument("name")

    args = parser.parse_args()

    if args.subcommand == "arm":
        command = {
            "command": "move_arm",
            "side": args.side,
            "positions": args.positions,
        }

    elif args.subcommand == "release":
        command = {
            "command": "release_arm",
            "side": args.side,
        }

    elif args.subcommand == "joints":
        command = {
            "command": "list_joints",
        }

    elif args.subcommand == "robot":
        command = {
            "command": "move_robot_root",
            "position": [args.x, args.y, args.z],
        }

    elif args.subcommand == "scene":
        if args.scene_name == "tabletop":
            command = {
                "command": "load_tabletop_scene",
            }
        else:
            command = {
                "command": "load_shelf_reach_scene",
            }

    elif args.subcommand == "spawn":
        command = {
            "command": "spawn_usd",
            "name": args.name,
            "path": args.path,
            "position": [args.x, args.y, args.z],
            "scale": [args.scale, args.scale, args.scale],
        }

    elif args.subcommand == "move":
        command = {
            "command": "move_object",
            "name": args.name,
            "position": [args.x, args.y, args.z],
        }

    elif args.subcommand == "remove":
        command = {
            "command": "remove",
            "name": args.name,
        }

    else:
        parser.error(f"Unsupported command: {args.subcommand}")
        return

    try:
        send_command(command)
    except OSError as exc:
        print(
            f"Failed to connect to runtime commander at {HOST}:{PORT}: {exc}",
            file=sys.stderr,
        )
        sys.exit(1)


if __name__ == "__main__":
    main()
