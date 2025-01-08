use pyo3::prelude::*;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

/// Formats the sum of two numbers as string.
#[pyfunction]
fn sum_as_string(a: usize, b: usize) -> PyResult<usize> {
    Ok((a + b))
}

/// Test Sequence
#[pyfunction]
fn test_sequence() -> PyResult<Vec<(i32, i32)>> {
    Ok(vec![(1, 2), (3, 4)])
}

/// read maze
#[pyfunction]
fn read_maze(_maze: Vec<Vec<usize>>) -> PyResult<String> {
    let mut result = String::new();

    for row in _maze {
        for num in row {
            result.push_str(&format!("{} ", num));
        }
        result.push('\n');
    }

    Ok(result)
}

#[derive(Debug, Copy, Clone, Eq, PartialEq, Hash)]
struct Position {
    x: i32,
    y: i32,
}

#[derive(Debug, Copy, Clone)]
struct Node {
    f_score: f64,
    g_score: f64,
    position: Position,
}

// Custom ordering for Node to work with BinaryHeap
impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        self.f_score
            .partial_cmp(&other.f_score)
            .unwrap_or(Ordering::Equal)
            .reverse() // BinaryHeap is a max-heap, so we reverse for min-heap behavior
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
    }
}

impl Eq for Node {}

#[pyfunction]
fn astar(
    maze: Vec<Vec<usize>>,
    start_x: i32,
    start_y: i32,
    goal_x: i32,
    goal_y: i32,
) -> PyResult<Vec<(i32, i32)>> {
    let start = Position {
        x: start_x,
        y: start_y,
    };
    let goal = Position {
        x: goal_x,
        y: goal_y,
    };

    if let Some(path) = find_path(&maze, start, goal) {
        let path_tuples = path.into_iter().map(|pos| (pos.x, pos.y)).collect();
        Ok(path_tuples)
    } else {
        Ok(vec![])
    }
}

fn find_path(maze: &Vec<Vec<usize>>, start: Position, goal: Position) -> Option<Vec<Position>> {
    let mut open_queue = BinaryHeap::new();
    let mut open_set = HashSet::new();
    let mut g_scores = HashMap::new();
    let mut came_from = HashMap::new();

    // Initialize start node
    g_scores.insert(start, 0.0);
    let start_node = Node {
        f_score: euclidean_distance_squared(start, goal),
        g_score: 0.0,
        position: start,
    };
    open_queue.push(start_node);
    open_set.insert(start);

    while let Some(current) = open_queue.pop() {
        let current_pos = current.position;
        open_set.remove(&current_pos);

        if current_pos == goal {
            return Some(reconstruct_path(&came_from, current_pos, start));
        }

        if current.g_score > *g_scores.get(&current_pos).unwrap_or(&f64::INFINITY) {
            continue;
        }

        for (next_pos, cost) in get_neighbors(maze, current_pos) {
            let tentative_g = g_scores.get(&current_pos).unwrap_or(&f64::INFINITY) + cost;

            if tentative_g < *g_scores.get(&next_pos).unwrap_or(&f64::INFINITY) {
                came_from.insert(next_pos, current_pos);
                g_scores.insert(next_pos, tentative_g);
                let f_score = tentative_g + euclidean_distance_squared(next_pos, goal);

                if !open_set.contains(&next_pos) {
                    let next_node = Node {
                        f_score,
                        g_score: tentative_g,
                        position: next_pos,
                    };
                    open_queue.push(next_node);
                    open_set.insert(next_pos);
                }
            }
        }
    }

    None
}

fn euclidean_distance_squared(pos1: Position, pos2: Position) -> f64 {
    let dx = (pos1.x - pos2.x) as f64;
    let dy = (pos1.y - pos2.y) as f64;
    dx * dx + dy * dy
}

fn get_neighbors(maze: &Vec<Vec<usize>>, pos: Position) -> Vec<(Position, f64)> {
    let directions = [(0, 1), (1, 0), (0, -1), (-1, 0)];
    let mut neighbors = Vec::new();

    for (dx, dy) in directions.iter() {
        let nx = pos.x + dx;
        let ny = pos.y + dy;

        if nx >= 0 && (nx as usize) < maze.len() && ny >= 0 && (ny as usize) < maze[0].len() {
            if maze[nx as usize][ny as usize] == 1 {
                neighbors.push((Position { x: nx, y: ny }, 1.0));
            }
        }
    }

    neighbors
}

fn reconstruct_path(
    came_from: &HashMap<Position, Position>,
    mut current: Position,
    _start: Position,
) -> Vec<Position> {
    let mut path = vec![current];
    while let Some(&prev) = came_from.get(&current) {
        path.push(prev);
        current = prev;
    }
    path.reverse();
    path
}

/// A Python module implemented in Rust.
#[pymodule]
fn stara_rs(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(sum_as_string, m)?)?;
    m.add_function(wrap_pyfunction!(read_maze, m)?)?;
    m.add_function(wrap_pyfunction!(astar, m)?)?;
    m.add_function(wrap_pyfunction!(test_sequence, m)?)?;
    Ok(())
}
