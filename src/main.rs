extern crate core;

use itertools::Itertools;
use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, HashSet, VecDeque};
use std::fmt::{Display, Formatter};

use search::*;
use std::io::{read_to_string, stdin, BufRead};
use std::rc::Rc;
use std::time::{Duration, Instant};
use tabled::builder::Builder;
use tabled::{Style, Table, Tabled};

fn main() {
    let state = [1_u8, 2, 5, 3, 4, 0, 6, 7, 8];
    // let state = (0..9_u8).permutations(9).take(100);
    // let state = get_state_from_user();
    let fns = [dfs, bfs, Astar_euclid, Astar_manhattan];
    // let mut results = vec![];
    // for func in fns {
    //     let res = state
    //         .clone()
    //         .filter_map(|s| {
    //             // dbg!(&s);
    //             func(&s).0
    //         })
    //         .map(|n| n.depth as usize)
    //         .collect_vec();
    //     let avg = res.iter().sum::<usize>() / res.len();
    //     results.push(avg);
    // }

    // let fns = ["DFS", "BFS", "A* with Euclid", "A* with Manhattan"];
    // for (name, val) in fns.iter().zip(results.iter()) {
    //     println!("{} : {}", name, val);
    // }
    let results = fns.map(|f| f(&state));
    let fns = ["DFS", "BFS", "A* with Euclid", "A* with Manhattan"];
    for (i, (node, gen_count, num_states, expanded)) in results.into_iter().enumerate() {
        let name = fns[i];
        let node = node.unwrap();
        println!("Method :{}\nNumber of Unique States Generated:{}\nTotal Number of States Generated:{}\nNumber of Nodes Expanded:{}\nSolution found at depth and with Cost:{}",name,gen_count,num_states,expanded,node.depth);
        println!("Solution Path:");
        let path = backtrack_goal_to_root(node);
        let path = path_diagram(&path);
        println!("{path}\n-----------------------------------------------------------------------");
    }
}

// type SearchFn = fn(&[u8]) -> (Option<Rc<Node>>, usize);
//
// // #[inline(always)]
// fn timeit(func: SearchFn, input: &[u8]) -> ((Option<Rc<Node>>, usize), Duration) {
//     let start = Instant::now();
//     let result = func(input);
//     (result, start.elapsed())
// }
