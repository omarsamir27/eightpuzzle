use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, HashSet, VecDeque};
use std::rc::Rc;
use std::thread::sleep;
use std::time::Duration;
use tabled::{Style, Table, Tabled};

fn main() {
    let state = [1_u8, 2, 5, 3, 4, 0, 6, 7, 8];
    let res = Astar_euclid(state.as_slice());
    let res =res.unwrap();
    println!("goal at depth {}",res.depth);
    let track = backtrack_goal_to_root(res);
    for s in track{
        print!("{esc}[2J{esc}[1;1H", esc = 27 as char);
        println!("{}",s.to_table());
        sleep(Duration::from_millis(500));
    }
    let state = State::new(state.as_slice());
    println!("{}", state.calculate_euclid());
}

fn backtrack_goal_to_root(node: Rc<Node>) -> Vec<State> {
    let mut states = vec![];
    let mut current = node.as_ref();
    dbg!(current.state.calculate_manhattan());
    states.push(current.state.clone());
    while let Some(parent) = current.parent.as_ref() {
        states.push(parent.state.clone());
        current = parent;
        dbg!(current.state.calculate_manhattan());
    }
    states.reverse();
    states
}

fn Astar_euclid(state: &[u8]) -> Option<Rc<Node>> {
    let root = Rc::new(Node::new(state, None, 0));
    let mut frontier = BinaryHeap::with_capacity(20);
    let mut visited = HashSet::with_capacity(20);
    frontier.push(Reverse(Euclid(
        root.depth as f32 + root.state.calculate_euclid(),
        root
    )));
    while let Some(Reverse(Euclid(_, node))) = frontier.pop() {
        if node.state.is_goal() {
            return Some(node);
        }
        let child_states = node.get_children_states();
        let children = child_states
            .iter()
            .filter(|s| visited.insert(s.grid))
            .map(|state| {
                Rc::new(Node {
                    state: state.clone(),
                    parent: Some(node.clone()),
                    depth: node.depth + 1,
                })
            })
            .map(|node|
                Reverse(Euclid(node.depth as f32 + node.state.calculate_euclid() , node)));
        frontier.extend(children)
    }
    None
}


fn Astar_manhattan(state: &[u8]) -> Option<Rc<Node>> {
    let root = Rc::new(Node::new(state, None, 0));
    let mut frontier = BinaryHeap::with_capacity(20);
    let mut visited = HashSet::with_capacity(20);
    frontier.push(Reverse(Manhattan(
        root.depth + root.state.calculate_manhattan(),
        root
    )));
    while let Some(Reverse(Manhattan(_, node))) = frontier.pop() {
        if node.state.is_goal() {
            return Some(node);
        }
        let child_states = node.get_children_states();
        let children = child_states
            .iter()
            .filter(|s| visited.insert(s.grid))
            .map(|state| {
                Rc::new(Node {
                    state: state.clone(),
                    parent: Some(node.clone()),
                    depth: node.depth + 1,
                })
            })
            .map(|node|
                Reverse(Manhattan(node.depth + node.state.calculate_manhattan(), node)));
        frontier.extend(children)
    }
    None
}
fn bfs(state: &[u8]) -> Option<Rc<Node>> {
    let root = Rc::new(Node::new(state, None, 0));
    let mut frontier = VecDeque::with_capacity(20);
    let mut visited = HashSet::with_capacity(20);
    frontier.push_back(root);
    while let Some(node) = frontier.pop_front() {
        if node.state.is_goal() {
            return Some(node);
        }
        let child_states = node.get_children_states();
        let children = child_states
            .iter()
            .filter(|s| visited.insert(s.grid))
            .map(|state| {
                Rc::new(Node {
                    state: state.clone(),
                    parent: Some(node.clone()),
                    depth: node.depth + 1,
                })
            });
        frontier.extend(children);
    }
    None
}
fn dfs(state: &[u8]) -> Option<Rc<Node>> {
    let root = Rc::new(Node::new(state, None, 0));
    let mut frontier = VecDeque::with_capacity(20);
    let mut visited = HashSet::with_capacity(20);
    frontier.push_back(root);
    while let Some(node) = frontier.pop_front() {
        if node.state.is_goal() {
            return Some(node);
        }
        let child_states = node.get_children_states();
        let children = child_states
            .iter()
            .filter(|s| visited.insert(s.grid))
            .map(|state| {
                Rc::new(Node {
                    state: state.clone(),
                    parent: Some(node.clone()),
                    depth: node.depth + 1,
                })
            });
        for child in children {
            frontier.push_front(child);
        }
    }
    None
}


struct Euclid(f32,Rc<Node>);

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

struct Manhattan(u8, Rc<Node>);

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

#[derive(Debug)]
struct Node {
    state: State,
    parent: Option<Rc<Node>>,
    depth: u8,
}

impl Node {
    fn new(state: &[u8], parent: Option<Rc<Node>>, depth: u8) -> Self {
        Node {
            state: State::new(state),
            parent,
            depth,
        }
    }
    fn get_children_states(&self) -> Vec<State> {
        self.state.evolution()
    }
}

#[derive(Clone, Debug)]
struct State {
    grid: [u8; 9],
}
impl State {
    fn digit_goal_pos(digit: u8) -> (u8, u8) {
        (digit % 3, digit / 3)
    }
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
    fn is_goal(&self) -> bool {
        self.grid == [0, 1, 2, 3, 4, 5, 6, 7, 8]
    }
    fn new(state: &[u8]) -> Self {
        Self {
            grid: state.try_into().unwrap(),
        }
    }
    fn evolution(&self) -> Vec<Self> {
        let mut ret = Vec::with_capacity(4);
        if let Some(left) = self.evolve_left() {
            ret.push(left)
        };
        if let Some(right) = self.evolve_right() {
            ret.push(right)
        };
        if let Some(up) = self.evolve_up() {
            ret.push(up)
        };
        if let Some(down) = self.evolve_down() {
            ret.push(down)
        };
        ret
    }
    fn evolve_left(&self) -> Option<Self> {
        let blank_pos = self.grid.iter().position(|d| *d == 0).unwrap();
        if matches!(blank_pos, 0 | 3 | 6) {
            return None;
        }
        let mut grid = self.grid;
        grid.swap(blank_pos, blank_pos - 1);
        Some(Self {
            grid,
        })
    }
    fn evolve_right(&self) -> Option<Self> {
        let blank_pos = self.grid.iter().position(|d| *d == 0).unwrap();
        if matches!(blank_pos, 2 | 5 | 8) {
            return None;
        }
        let mut grid = self.grid;
        grid.swap(blank_pos, blank_pos + 1);
        Some(Self {
            grid,
        })
    }
    fn evolve_up(&self) -> Option<Self> {
        let blank_pos = self.grid.iter().position(|d| *d == 0).unwrap();
        if matches!(blank_pos, 0 | 1 | 2) {
            return None;
        }
        let mut grid = self.grid;
        grid.swap(blank_pos, blank_pos - 3);
        Some(Self {
            grid,
        })
    }
    fn evolve_down(&self) -> Option<Self> {
        let blank_pos = self.grid.iter().position(|d| *d == 0).unwrap();
        if matches!(blank_pos, 6 | 7 | 8) {
            return None;
        }
        let mut grid = self.grid;
        grid.swap(blank_pos, blank_pos + 3);
        Some(Self {
            grid,
        })
    }
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
