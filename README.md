# NOSBENCH
`NOSBENCH` is a benchmark suite of mathematical programs with complementarity constraiants (MPCCs):
```
minimize f(w,p), subject to
lbw <= w <= ubw
lbg <= g(w,p) <= ubg
0 <= G(w,p) perp H(w,p) >= 0
```
We provide the problems in two forms: casADi expressions stored in json encoded structures and matlab `.mat` files which can be used with `v0.5.0` of [`nosnoc`](https://github.com/nurkanovic/nosnoc).


## Literature
[Solving mathematical programs with complementarity constraints arising in nonsmooth optimal control](https://arxiv.org/abs/2312.11022) \
A. Nurkanović, A. Pozharskiy, M. Diehl \
arXiv preprint 2023
```
@article{Nurkanovic2023,
      title={Solving mathematical programs with complementarity constraints arising in nonsmooth optimal control}, 
      author={Armin Nurkanović and Anton Pozharskiy and Moritz Diehl},
      year={2023},
      eprint={2312.11022},
      archivePrefix={arXiv},
      primaryClass={math.OC}
}
```
