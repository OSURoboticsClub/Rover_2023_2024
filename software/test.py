from rich.live import Live
from rich.table import Table
from time import sleep
import random

def generate_table(data):
    print("Call")
    table = Table(title="Live Updating Table")
    table.add_column("ID", style="cyan", no_wrap=True)
    table.add_column("Value", style="magenta")
    for row in data:
        table.add_row(str(row["id"]), str(row["value"]))
    return table

data = [{"id": i, "value": 0} for i in range(5)]

with Live(generate_table(data), refresh_per_second=4) as live:
    for _ in range(100):
        # Update data
        for row in data:
            row["value"] = random.randint(0, 100)
        live.update(generate_table(data))
        sleep(0.5)
