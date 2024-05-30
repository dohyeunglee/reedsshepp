# reedsshepp

> Pure Golang implementation of Reeds Shepp Path, based on C++ [ompl](https://github.com/ompl/ompl) ReedsSheppState

## Installation
```bash
go get github.com/dohyeunglee/reedsshepp
```

## API
Visit https://pkg.go.dev/github.com/dohyeunglee/reedsshepp

## Example
An example is available in [example/main.go](example/main.go)

## Demo
A demo with plot is available in [demo/main.go](demo/main.go)

## Test
```bash
go test
```

## Benchmark
```bash
go test -bench .
```
### Result
Run on M1 Macbook Pro
```bash
BenchmarkMinLengthPath-10    	  225084	      5020 ns/op
```


## Note
Due to the difference in the implementation of mathematical functions like sin, cos between C++ and Golang,
accumulated errors in floating-point calculations can lead to differences between the results of the ompl library and `AvailablePaths`.

