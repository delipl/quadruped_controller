import subprocess

def run_command():
    numbers = [10, 11, 12, 20, 21, 22, 30, 31, 32, 40, 41, 42]
    for number in numbers:
        command = f"mdtool config zero {number}"
        subprocess.run(command, shell=True)

if __name__ == "__main__":
    run_command()