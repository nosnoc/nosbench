# NOSBENCH
`NOSBENCH` is a benchmark suite of mathematical programs with complementarity constraiants (MPCCs):
```
minimize f(w,p), subject to
lbw <= w <= ubw
lbg <= g(w,p) <= ubg
0 <= G(w,p) perp H(w,p) >= 0
```
We provide the problems in two forms: casADi expressions stored in json encoded structures and matlab `.mat` files which can be used with `v0.5.0` of [`nosnoc`](https://github.com/nurkanovic/nosnoc).

The json structures contains the fields: `w`, `lbw`, `ubw`, `w0`, `p`, `p0`, `g_fun(w,p)` (general nonlinear constriants `g`), `lbg`, `ubg`, `G_fun(w,p)`, `H_fun(w,p)`, `augmented_objective_fun(w,p)` (objective `f(w,p)`).

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

[Evaluating Methods for Solving Mathematical Programs With Complementarity Constraints Arising From Nonsmooth Optimal Control](https://publications.syscop.de/Pozharskiy2023.pdf) \
A. Pozharskiy
Master's Thesis
```
@mastersthesis{Pozharskiy2023,
	keywords = {syscop-public},
	year = {2023},
	school = {Albert-Ludwigs-University Freiburg},
	author = {Anton Pozharskiy},
	title = {Evaluating Methods for Solving Mathematical Programs With Complementarity Constraints Arising From Nonsmooth Optimal Control.},
}
```
