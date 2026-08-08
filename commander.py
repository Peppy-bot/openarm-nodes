#!/usr/bin/env python3

import argparse
import json
import os
import socket


HOST = os.getenv(
    "PEPPY_ISAAC_COMMAND_HOST",
    "127.0.0.1",
)

PORT = int(
    os.getenv(
        "PEPPY_ISAAC_COMMAND_PORT",
        "5556",
    )
)


def send(command):
    message = (
        json.dumps(command) + "\n"
    ).encode("utf-8")

    with socket.create_connection(
        (HOST, PORT),
        timeout=5,
    ) as sock:
        sock.sendall(message)
        reply = sock.recv(65536)
        print(
            reply.decode("utf-8").strip()
        )


parser = argparse.ArgumentParser(
    description="Runtime commander for OpenArm Isaac Sim"
)

sub = parser.add_subparsers(
    dest="command",
    required=True,
)

arm = sub.add_parser(
    "arm",
    help="Command one 7-DOF arm",
)

arm.add_argument(
    "side",
    choices=["left", "right"],
)

arm.add_argument(
    "positions",
    type=float,
    nargs=7,
)

release = sub.add_parser(
    "release",
    help="Release runtime arm control back to Peppy",
)

release.add_argument(
    "side",
    choices=["left", "right"],
)

sub.add_parser(
    "joints",
    help="Print OpenArm joint names in the Isaac log",
)

robot = sub.add_parser(
    "robot",
    help="Move the complete OpenArm root",
)

robot.add_argument("x", type=float)
robot.add_argument("y", type=float)
robot.add_argument("z", type=float)

scene = sub.add_parser(
    "scene",
    help="Load a runtime scene preset",
)

scene.add_argument(
    "preset",
    choices=[
        "tabletop",
        "shelf_reach",
    ],
)

spawn = sub.add_parser(
    "spawn",
    help="Spawn a USD asset into the live Isaac stage",
)

spawn.add_argument("name")
spawn.add_argument("path")
spawn.add_argument("x", type=float)
spawn.add_argument("y", type=float)
spawn.add_argument("z", type=float)

spawn.add_argument(
    "--scale",
    type=float,
    default=1.0,
)

move = sub.add_parser(
    "move",
    help="Move a runtime USD object",
)

move.add_argument("name")
move.add_argument("x", type=float)
move.add_argument("y", type=float)
move.add_argument("z", type=float)

remove = sub.add_parser(
    "remove",
    help="Remove a runtime USD object",
)

remove.add_argument("name")

args = parser.parse_args()


if args.command == "arm":
    send({
        "command": "move_arm",
        "side": args.side,
        "positions": args.positions,
    })

elif args.command == "release":
    send({
        "command": "release_arm",
        "side": args.side,
    })

elif args.command == "joints":
    send({
        "command": "list_joints",
    })

elif args.command == "robot":
    send({
        "command": "move_robot_root",
        "position": [
            args.x,
            args.y,
            args.z,
        ],
    })

elif args.command == "scene":
    if args.preset == "tabletop":
        send({
            "command": "load_tabletop_scene",
        })

    elif args.preset == "shelf_reach":
        send({
            "command": "load_shelf_reach_scene",
        })

elif args.command == "spawn":
    send({
        "command": "spawn_usd",
        "name": args.name,
        "path": args.path,
        "position": [
            args.x,
            args.y,
            args.z,
        ],
        "scale": [
            args.scale,
            args.scale,
            args.scale,
        ],
    })

elif args.command == "move":
    send({
        "command": "move_object",
        "name": args.name,
        "position": [
            args.x,
            args.y,
            args.z,
        ],
    })

elif args.command == "remove":
    send({
        "command": "remove",
        "name": args.name,
    })
