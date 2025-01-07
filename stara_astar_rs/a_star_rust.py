from typing import Tuple, List, Optional

from numpy._typing import NDArray
from stara_maze_generator.pathfinder.base import PathfinderBase
from stara_rs.stara_rs import astar


class AStarRust(PathfinderBase):
    def __init__(self, maze):
        super().__init__(maze)
        self.maze_list: List[List[int]] = [list(row) for row in self.maze.maze_map]

    def find_path(
        self, start: NDArray | Tuple[int, int], goal: NDArray | Tuple[int, int]
    ) -> Optional[List[Tuple[int, int]]]:
        return astar(self.maze_list, start[0], start[1], goal[0], goal[1])
