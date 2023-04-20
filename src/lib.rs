use crate::Move::*;
use itertools::Itertools;
use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, HashSet, VecDeque};
use std::fmt::{Display, Formatter};
use std::io::{stdin, BufRead};
use std::rc::Rc;
use tabled::builder::Builder;
use tabled::{Style, Table, Tabled};

pub fn get_state_from_user() -> Vec<u8> {
    let mut str = String::new();
    stdin().lock().read_line(&mut str   ).unwrap();
    str.remove(str.len() - 1);
    str.split_terminator(',')
        .map(|num| num.parse::<u8>().unwrap())
        .collect_vec()
}

/// Visualize the state transition tree that lead to the goal and the movement that generated it
pub fn path_diagram(path: &[(State, Move)]) -> String {
    let mut builder = Builder::default();
    builder.set_columns(["Step", "State", "Move"]);
    for (i, (s, m)) in path.iter().enumerate() {
        builder.add_record([i.to_string(), s.to_table(), m.to_string()]);
    }
    builder.build().to_string()
}
/// Backtrack the reached goal state to the root node with the movements that generated each step
pub fn backtrack_goal_to_root(node: Rc<Node>) -> Vec<(State, Move)> {
    let mut states = vec![];
    let mut current = node.as_ref();
    states.push((current.state.clone(), current.creating_move));
    while let Some(parent) = current.parent.as_ref() {
        states.push((parent.state.clone(), parent.creating_move));
        current = parent;
    }
    states.reverse();
    states
}
/// Informed A* Search using the Euclid Heurisitc, The Euclidean heuristic uses the straight line distance
/// between each tile and its goal location
pub fn Astar_euclid(state: &[u8]) -> (Option<Rc<Node>>, usize, u64, u64) {
    let mut state_counter = 0;
    let mut expanded_counter = 0;
    let root = Rc::new(Node::new(state, None, 0, NoOp));
    let mut frontier = BinaryHeap::new();
    let mut visited = HashSet::new();
    frontier.push(Reverse(Euclid(
        root.depth as f32 + root.state.calculate_euclid(),
        root,
    )));
    while let Some(Reverse(Euclid(_, node))) = frontier.pop() {
        expanded_counter += 1;
        if node.state.is_goal() {
            return (Some(node), visited.len(), state_counter, expanded_counter);
        }
        let child_states = node.get_children_states();
        state_counter += child_states.len() as u64;
        let children = child_states
            .iter()
            .filter(|(s, m)| visited.insert(s.grid))
            .map(|(s, m)| {
                Rc::new(Node {
                    state: s.clone(),
                    parent: Some(node.clone()),
                    depth: node.depth + 1,
                    creating_move: *m,
                })
            })
            .map(|node| {
                Reverse(Euclid(
                    node.depth as f32 + node.state.calculate_euclid(),
                    node,
                ))
            });
        frontier.extend(children)
    }
    (None, visited.len(), state_counter, expanded_counter)
}
/// Informed A* Search using the Manhattan Heurisitc, The Manhattan heuristic uses the number of horizontal
/// and vertical moves needed for a tile to reach its final location
pub fn Astar_manhattan(state: &[u8]) -> (Option<Rc<Node>>, usize, u64, u64) {
    let mut state_counter = 0;
    let mut expanded_counter = 0;
    let root = Rc::new(Node::new(state, None, 0, NoOp));
    let mut frontier = BinaryHeap::new();
    let mut visited = HashSet::new();
    frontier.push(Reverse(Manhattan(
        root.depth + root.state.calculate_manhattan() as u64,
        root,
    )));
    while let Some(Reverse(Manhattan(_, node))) = frontier.pop() {
        expanded_counter += 1;
        if node.state.is_goal() {
            return (Some(node), visited.len(), state_counter, expanded_counter);
        }
        let child_states = node.get_children_states();
        state_counter += child_states.len() as u64;
        let children = child_states
            .iter()
            .filter(|(s, m)| visited.insert(s.grid))
            .map(|(s, m)| {
                Rc::new(Node {
                    state: s.clone(),
                    parent: Some(node.clone()),
                    depth: node.depth + 1,
                    creating_move: *m,
                })
            })
            .map(|node| {
                Reverse(Manhattan(
                    node.depth + node.state.calculate_manhattan() as u64,
                    node,
                ))
            });
        frontier.extend(children)
    }
    (None, visited.len(), state_counter, expanded_counter)
}
/// Blind Search using Breadth First Search , loops are avoided by not generating previously generated states
pub fn bfs(state: &[u8]) -> (Option<Rc<Node>>, usize, u64, u64) {
    let mut state_counter = 0;
    let mut expanded_counter = 0;
    let root = Rc::new(Node::new(state, None, 0, NoOp));
    let mut frontier = VecDeque::new();
    let mut visited = HashSet::new();
    frontier.push_back(root);
    while let Some(node) = frontier.pop_front() {
        expanded_counter += 1;
        if node.state.is_goal() {
            return (Some(node), visited.len(), state_counter, expanded_counter);
        }
        let child_states = node.get_children_states();
        state_counter += child_states.len() as u64;
        let children = child_states
            .iter()
            .filter(|(s, m)| visited.insert(s.grid))
            .map(|(s, m)| {
                Rc::new(Node {
                    state: s.clone(),
                    parent: Some(node.clone()),
                    depth: node.depth + 1,
                    creating_move: *m,
                })
            });
        // for child in children{
        //     frontier.push_back(child);
        // }
        frontier.extend(children);
    }
    (None, visited.len(), state_counter, expanded_counter)
}
/// Blind Search using Depth First Search , loops are avoided by not generating previously generated states
pub fn dfs(state: &[u8]) -> (Option<Rc<Node>>, usize, u64, u64) {
    let mut state_counter = 0;
    let mut expanded_counter = 0;
    let root = Rc::new(Node::new(state, None, 0, NoOp));
    let mut frontier = VecDeque::new();
    let mut visited = HashSet::new();
    frontier.push_back(root);
    while let Some(node) = frontier.pop_front() {
        expanded_counter +=1;
        if node.state.is_goal() {
            return (Some(node), visited.len(), state_counter, expanded_counter);
        }
        let child_states = node.get_children_states();
        state_counter += child_states.len() as u64;
        let children = child_states
            .iter()
            .filter(|(s, m)| visited.insert(s.grid))
            .map(|(s, m)| {
                Rc::new(Node {
                    state: s.clone(),
                    parent: Some(node.clone()),
                    depth: node.depth + 1,
                    creating_move: *m,
                })
            });
        for child in children {
            frontier.push_front(child);
        }
    }
    (None, visited.len(), state_counter, expanded_counter)
}
/// Wrapper Struct to use Euclid Heuristic to sort nodes in a priority queue.
struct Euclid(f32, Rc<Node>);

impl Eq for Euclid {}

impl PartialEq<Self> for Euclid {
    fn eq(&self, other: &Self) -> bool {
        self.0.eq(&other.0)
    }
}

impl PartialOrd<Self> for Euclid {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.0.partial_cmp(&other.0)
    }
}

impl Ord for Euclid {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.total_cmp(&other.0)
    }
}
/// Wrapper Struct to use Manhattan Heuristic to sort nodes in a priority queue.
struct Manhattan(u64, Rc<Node>);

impl Eq for Manhattan {}

impl PartialEq<Self> for Manhattan {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl PartialOrd<Self> for Manhattan {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.0.partial_cmp(&other.0)
    }
}

impl Ord for Manhattan {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.cmp(&other.0)
    }
}
/// Represents a node in the state tree
#[derive(Debug)]
pub struct Node {
    state: State,
    parent: Option<Rc<Node>>,
    pub depth: u64,
    creating_move: Move,
}

impl Node {
    fn new(state: &[u8], parent: Option<Rc<Node>>, depth: u64, creating_move: Move) -> Self {
        Node {
            state: State::new(state),
            parent,
            depth,
            creating_move,
        }
    }
    fn get_children_states(&self) -> Vec<(State, Move)> {
        self.state.evolution()
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Move {
    Up,
    Down,
    Left,
    Right,
    NoOp,
}

impl Display for Move {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let arrow = match self {
            Move::Up => '↑',
            Move::Down => '↓',
            Move::Left => '←',
            Move::Right => '→',
            Move::NoOp => ' ',
        };
        write!(f, "{}", arrow)
    }
}
/// A state that represents the order of the tiles from 0 to 8 in row-major order
#[derive(Clone, Debug)]
pub struct State {
    grid: [u8; 9],
}
impl State {
    #[inline(always)]
    fn digit_goal_pos(digit: u8) -> (u8, u8) {
        (digit % 3, digit / 3)
    }
    /// Calculate the manhattan heurisitc for this state by summing the manhattan distance for every non-zero
    /// tile , the manhattan distance is the number of vertical and horizontal moves a tile needs to reach
    /// its goal position
    #[inline(always)]
    fn calculate_manhattan(&self) -> u8 {
        let mut sum = 0;
        for (pos, elem) in self.grid.iter().enumerate() {
            if *elem != 0 && pos != *elem as usize {
                let goal = Self::digit_goal_pos(*elem);
                let pos = Self::digit_goal_pos(pos as u8);
                sum += i8::abs(goal.0 as i8 - pos.0 as i8) + i8::abs(goal.1 as i8 - pos.1 as i8);
            }
        }
        sum as u8
    }
    /// Calculate the euclid heurisitc for this state by summing the straight line for every non-zero
    /// tile , the euclidean distance is the length of the hypotenuse from a tile position to its goal
    /// position
    #[inline(always)]
    fn calculate_euclid(&self) -> f32 {
        let mut sum = 0_f32;
        for (pos, elem) in self.grid.iter().enumerate() {
            if *elem != 0 && pos != *elem as usize {
                let goal = Self::digit_goal_pos(*elem);
                let pos = Self::digit_goal_pos(pos as u8);
                let (dx, dy) = (goal.0 as i8 - pos.0 as i8, goal.1 as i8 - pos.1 as i8);
                sum += ((dx * dx + dy * dy) as f32).sqrt();
            }
        }
        sum
    }
    #[inline(always)]
    fn is_goal(&self) -> bool {
        self.grid == [0, 1, 2, 3, 4, 5, 6, 7, 8]
    }
    fn new(state: &[u8]) -> Self {
        Self {
            grid: state.try_into().unwrap(),
        }
    }
    #[inline(always)]
    fn evolution(&self) -> Vec<(Self, Move)> {
        let mut ret = Vec::with_capacity(4);
        if let Some(left) = self.evolve_left() {
            ret.push((left, Left))
        };
        if let Some(right) = self.evolve_right() {
            ret.push((right, Right))
        };
        if let Some(up) = self.evolve_up() {
            ret.push((up, Up))
        };
        if let Some(down) = self.evolve_down() {
            ret.push((down, Down))
        };
        ret
    }
    /// Try to move the blank tile 1 position to the left
    #[inline(always)]
    fn evolve_left(&self) -> Option<Self> {
        let blank_pos = self.grid.iter().position(|d| *d == 0).unwrap();
        if matches!(blank_pos, 0 | 3 | 6) {
            return None;
        }
        let mut grid = self.grid;
        grid.swap(blank_pos, blank_pos - 1);
        Some(Self { grid })
    }
    /// Try to move the blank tile 1 position to the right
    #[inline(always)]
    fn evolve_right(&self) -> Option<Self> {
        let blank_pos = self.grid.iter().position(|d| *d == 0).unwrap();
        if matches!(blank_pos, 2 | 5 | 8) {
            return None;
        }
        let mut grid = self.grid;
        grid.swap(blank_pos, blank_pos + 1);
        Some(Self { grid })
    }
    /// Try to move the blank tile 1 position up
    #[inline(always)]
    fn evolve_up(&self) -> Option<Self> {
        let blank_pos = self.grid.iter().position(|d| *d == 0).unwrap();
        if matches!(blank_pos, 0 | 1 | 2) {
            return None;
        }
        let mut grid = self.grid;
        grid.swap(blank_pos, blank_pos - 3);
        Some(Self { grid })
    }
    /// Try to move the blank tile 1 position down
    #[inline(always)]
    fn evolve_down(&self) -> Option<Self> {
        let blank_pos = self.grid.iter().position(|d| *d == 0).unwrap();
        if matches!(blank_pos, 6 | 7 | 8) {
            return None;
        }
        let mut grid = self.grid;
        grid.swap(blank_pos, blank_pos + 3);
        Some(Self { grid })
    }
    /// Pretty-Print a State
    #[inline(always)]
    fn to_table(&self) -> String {
        let data = [
            [self.grid[0], self.grid[1], self.grid[2]],
            [self.grid[3], self.grid[4], self.grid[5]],
            [self.grid[6], self.grid[7], self.grid[8]],
        ];
        table(data)
    }
}

fn table<I, T>(iter: I) -> String
where
    T: Tabled,
    I: IntoIterator<Item = T>,
{
    let mut t = Table::builder(iter);
    t.remove_columns();
    t.build().with(Style::modern()).to_string()
}
