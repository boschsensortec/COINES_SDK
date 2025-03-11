import argparse
from . import __version__


def main():
    parser = argparse.ArgumentParser(
        description="Command-line interface for coinespy Library"
    )
    parser.add_argument(
        "--version", action="version", version=f"coinespy {__version__}"
    )
    parser.parse_args()


if __name__ == "__main__":
    main()
