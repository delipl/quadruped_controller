import subprocess
import argparse


def run_command():
    leg_id = 4
    
    
    
    numbers = [leg_id*10, leg_id*10 + 1, leg_id*10 + 2]
    for number in numbers:
        command = f"mdtool config zero {number}"
        subprocess.run(command, shell=True)
        command = f"mdtool config save {number}"
        subprocess.run(command, shell=True)

if __name__ == "__main__":
    run_command()