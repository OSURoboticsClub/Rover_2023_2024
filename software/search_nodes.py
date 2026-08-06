import can

bus = can.interface.Bus(channel='can_cam', bustype='socketcan')

active_nodes = set()

while True:
    msg = bus.recv()

    if msg is None:
        continue

    arb_id = msg.arbitration_id

    cmd_id  = arb_id & 0x1F
    node_id = arb_id >> 5

    if cmd_id == 0x001:  # heartbeat
        active_nodes.add(node_id)

        print(f"Heartbeat from node {node_id}")
        print(f"Active nodes: {sorted(active_nodes)}")
