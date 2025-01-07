from typing import Tuple, List, Optional
import argparse
from time import time
from loguru import logger

from numpy.typing import NDArray

from stara_maze_generator.pathfinder.base import PathfinderBase
from stara_maze_generator.vmaze import VMaze
from stara_rs.stara_rs import astar


class AStarRust(PathfinderBase):
    def __init__(self, maze):
        super().__init__(maze)
        self.maze_list: List[List[int]] = [list(row) for row in self.maze.maze_map]

    def find_path(
        self, start: NDArray | Tuple[int, int], goal: NDArray | Tuple[int, int]
    ) -> Optional[List[Tuple[int, int]]]:
        return astar(self.maze_list, start[0], start[1], goal[0], goal[1])


if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument(
        "--file",
        type=str,
        help="Path to the maze file",
    )
    args = args.parse_args()
    with open(args.file) as f:
        maze = VMaze.from_json(f.read())
    pathfinder = AStarRust(maze)
    start_time = time()
    path = pathfinder.find_path(maze.start, maze.goal)
    end_time = time()
    if path is None:
        logger.error("No path found")
        exit(1)
    logger.info(f"Maze exported to {args.file}")
    logger.info([(int(x), int(y)) for (x, y) in path])
    logger.info(f"Path length: {len(path)}")
    logger.info(f"Time taken: {end_time - start_time} seconds")
