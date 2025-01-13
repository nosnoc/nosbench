# NOSBENCH
`NOSBENCH` is a benchmark suite of mathematical programs with complementarity constraiants (MPCCs):
```
minimize f(w,p), subject to
lbw <= w <= ubw
lbg <= g(w,p) <= ubg
0 <= G(w,p) perp H(w,p) >= 0
```
We provide the problems in two forms: casADi expressions stored in json encoded structures and matlab `.mat` files which can be used with `v0.5.0` of [`nosnoc`](https://github.com/nurkanovic/nosnoc).

The json structures contains the fields: `w`, `lbw`, `ubw`, `w0`, `p`, `p0`, `g_fun(w,p)` (general nonlinear constriants `g`), `lbg`, `ubg`, `G_fun(w,p)`, `H_fun(w,p)` (Complementarity varibles), `augmented_objective_fun(w,p)` (objective `f(w,p)`).

## Literature
[Solving mathematical programs with complementarity constraints arising in nonsmooth optimal control](https://link.springer.com/article/10.1007/s10013-024-00704-z) \
A. Nurkanović, A. Pozharskiy, M. Diehl \
Vietnam J. Math. (2024). 
```
@article{Nurkanovic2024,
  title={Solving mathematical programs with complementarity constraints arising in nonsmooth optimal control},
  author={Nurkanovi{\'c}, Armin and Pozharskiy, Anton and Diehl, Moritz},
  journal={Vietnam Journal of Mathematics},
  pages={1--39},
  year={2024},
  publisher={Springer}
}

```

[Evaluating Methods for Solving Mathematical Programs With Complementarity Constraints Arising From Nonsmooth Optimal Control](https://publications.syscop.de/Pozharskiy2023.pdf) \
A. Pozharskiy \
Master's Thesis
```
@mastersthesis{Pozharskiy2023,
	year = {2023},
	school = {Albert-Ludwigs-University Freiburg},
	author = {Anton Pozharskiy},
	title = {Evaluating Methods for Solving Mathematical Programs With Complementarity Constraints Arising From Nonsmooth Optimal Control.},
}
```
