import sys

if __name__ == "__main__":
    from src.PyIK import Kinectics
    app = Kinectics()
    while not app.stopped:
        app.tick()
