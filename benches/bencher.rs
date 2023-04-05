use std::fmt::Display;
use std::io::Read;
use std::time::Duration;
use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use itertools::Itertools;
use search::*;

fn criterion_benchmark(c: &mut Criterion) {
    let state = vec![vec![1_u8, 2, 5, 3, 4, 0, 6, 7, 8]];
    // let iter = (0..9_u8).permutations(9).take(3);
    let mut group = c.benchmark_group("Searchers");
    for (i,state) in state.iter().enumerate(){
        group.bench_with_input(BenchmarkId::new("DFS",vec2str(&state)),&state,|b,s| b.iter(|| dfs(black_box(s))));
        group.bench_with_input(BenchmarkId::new("BFS",vec2str(&state)),&state,|b,s| b.iter(|| bfs(black_box(s))));
        group.bench_with_input(BenchmarkId::new("A*Euclid",vec2str(&state)),&state,|b,s| b.iter(|| Astar_euclid(black_box(s))));
        group.bench_with_input(BenchmarkId::new("A*Manhattan",vec2str(&state)),&state,|b,s| b.iter(|| Astar_manhattan(black_box(s))));
    }

    group.finish();

}

fn vec2str<T:Display + ToString>(input:&[T]) -> String{
    let mut str = String::with_capacity(input.len());
    for c in input{
        str.push_str(c.to_string().as_str());
    }
    str
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);