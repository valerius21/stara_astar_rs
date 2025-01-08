use pyo3::prelude::*;

#[pyclass]
struct MazeSolver {
    width: usize,
    height: usize,
    data: Vec<u8>,           // Flat array for maze data: 0 = wall, 1 = path
    visited: Vec<u8>,        // Using u8 instead of bool for better SIMD
    parent: Vec<(i32, i32)>, // Store parent nodes for path reconstruction
}

#[pymethods]
impl MazeSolver {
    #[new]
    fn new() -> Self {
        MazeSolver {
            width: 0,
            height: 0,
            data: Vec::new(),
            visited: Vec::new(),
            parent: Vec::new(),
        }
    }

    fn load(&mut self, maze: Vec<Vec<usize>>) -> PyResult<()> {
        self.height = maze.len();
        self.width = maze[0].len();
        let size = self.width * self.height;

        self.data = Vec::with_capacity(size);
        self.visited = vec![0; size];
        self.parent = vec![(-1, -1); size];

        for row in maze {
            for cell in row {
                self.data.push(cell as u8);
            }
        }
        Ok(())
    }

    fn astar(&mut self, start: (i32, i32), goal: (i32, i32)) -> PyResult<Vec<(i32, i32)>> {
        if self.data.is_empty() {
            return Err(PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(
                "Maze not loaded",
            ));
        }

        let mut open = [(0i32, 0i32, 0i32); 1024];
        let mut open_len = 1;

        open[0] = (manhattan_distance(start, goal), start.0, start.1);
        self.visited.fill(0);
        self.parent.fill((-1, -1));

        while open_len > 0 {
            let mut min_idx = 0;
            let mut min_score = open[0].0;
            for i in 1..open_len {
                if open[i].0 < min_score {
                    min_idx = i;
                    min_score = open[i].0;
                }
            }

            let (_, x, y) = open[min_idx];
            open_len -= 1;
            open[min_idx] = open[open_len];

            if x == goal.0 && y == goal.1 {
                return Ok(self.reconstruct_path(start, (x, y)));
            }

            let current_idx = (x as usize) * self.width + (y as usize);
            self.visited[current_idx] = 1;

            static DIRS: [(i32, i32); 4] = [(0, 1), (1, 0), (0, -1), (-1, 0)];
            for &(dx, dy) in &DIRS {
                let nx = x + dx;
                let ny = y + dy;

                if nx >= 0 && (nx as usize) < self.height && ny >= 0 && (ny as usize) < self.width {
                    let idx = (nx as usize) * self.width + (ny as usize);
                    if self.visited[idx] == 0 && self.data[idx] == 1 {
                        self.parent[idx] = (x, y);
                        let f = manhattan_distance((nx, ny), goal);
                        if open_len < 1024 {
                            open[open_len] = (f, nx, ny);
                            open_len += 1;
                        }
                    }
                }
            }
        }
        Ok(vec![])
    }
}

impl MazeSolver {
    fn reconstruct_path(&self, start: (i32, i32), goal: (i32, i32)) -> Vec<(i32, i32)> {
        let mut path = Vec::new();
        let mut current = goal;

        while current != start {
            path.push(current);
            let idx = (current.0 as usize) * self.width + (current.1 as usize);
            current = self.parent[idx];
        }
        path.push(start);
        path.reverse();
        path
    }
}

#[inline(always)]
fn manhattan_distance(pos1: (i32, i32), pos2: (i32, i32)) -> i32 {
    (pos1.0 - pos2.0).abs() + (pos1.1 - pos2.1).abs()
}

#[pymodule]
fn stara_rs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<MazeSolver>()?;
    Ok(())
}
