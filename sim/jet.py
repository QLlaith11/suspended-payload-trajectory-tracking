"""Jet arithmetic: exact time derivatives through algebraic operations (step 2.1).

A Jet stores a quantity together with its time derivatives: d[k] = k-th derivative, k = 0..order, where each d[k]
can be a scalar, vector or matrix (numpy array of any shape). Operations propagate derivatives exactly:
    product rule (Leibniz):   (f g)^(n)  = sum_k C(n,k) f^(k) g^(n-k)
    reciprocal:               h = 1/g,  h^(n) = -(1/g) sum_{k=1..n} C(n,k) g^(k) h^(n-k)
    square root:              h = sqrt(g), h^(n) = (g^(n) - sum_{k=1..n-1} C(n,k) h^(k) h^(n-k)) / (2 h)
so any formula built from these operations has exact derivatives (up to floating-point round-off).
"""
from __future__ import annotations

from math import comb

import numpy as np


class Jet:
    __slots__ = ("d",)

    def __init__(self, derivs):
        self.d = np.asarray(derivs, dtype=float)

    # ------------------------------------------------------------------ basics
    @property
    def order(self) -> int:
        return self.d.shape[0] - 1

    @property
    def val(self) -> np.ndarray:
        return self.d[0]

    def __getitem__(self, k: int) -> np.ndarray:
        return self.d[k]

    def truncate(self, order: int) -> "Jet":
        return Jet(self.d[: order + 1])

    def shift(self, n: int) -> "Jet":
        """The n-th derivative as a jet (order decreases by n)."""
        return Jet(self.d[n:])

    def component(self, idx) -> "Jet":
        return Jet(self.d[(slice(None),) + np.index_exp[idx]])

    @staticmethod
    def constant(value, order: int) -> "Jet":
        v = np.asarray(value, dtype=float)
        d = np.zeros((order + 1,) + v.shape)
        d[0] = v
        return Jet(d)

    def _align(self, other):
        if not isinstance(other, Jet):
            other = Jet.constant(other, self.order)
        n = min(self.order, other.order)
        return self.truncate(n), other.truncate(n)

    # ------------------------------------------------------------------ arithmetic
    def __add__(self, other):
        a, b = self._align(other)
        return Jet(a.d + b.d)

    __radd__ = __add__

    def __neg__(self):
        return Jet(-self.d)

    def __sub__(self, other):
        return self + (-other if isinstance(other, Jet) else -np.asarray(other, float))

    def __rsub__(self, other):
        return (-self) + other

    def _leibniz(self, other, op):
        a, b = self._align(other)
        n = a.order
        out = [sum(comb(m, k) * op(a.d[k], b.d[m - k]) for k in range(m + 1)) for m in range(n + 1)]
        return Jet(np.array(out))

    def __mul__(self, other):
        """Elementwise product (with numpy broadcasting, e.g. scalar jet * vector jet)."""
        if not isinstance(other, Jet):
            return Jet(self.d * np.asarray(other, float))
        sa, sb = self.d.ndim - 1, other.d.ndim - 1
        if sa == 0 and sb > 0:          # scalar jet * array jet: broadcast scalar over the array
            return self._leibniz(other, lambda x, y: x * y)
        if sb == 0 and sa > 0:
            return other._leibniz(self, lambda x, y: x * y)
        return self._leibniz(other, lambda x, y: x * y)

    __rmul__ = __mul__

    def dot(self, other: "Jet") -> "Jet":
        return self._leibniz(other, lambda x, y: np.dot(x, y))

    def cross(self, other: "Jet") -> "Jet":
        return self._leibniz(other, lambda x, y: np.cross(x, y))

    def matmul(self, other: "Jet") -> "Jet":
        return self._leibniz(other, lambda x, y: x @ y)

    def T(self) -> "Jet":
        """Transpose of a matrix jet."""
        return Jet(np.swapaxes(self.d, -1, -2))

    def reciprocal(self) -> "Jet":
        g, n = self.d, self.order
        h = np.zeros_like(g)
        h[0] = 1.0 / g[0]
        for m in range(1, n + 1):
            h[m] = -h[0] * sum(comb(m, k) * g[k] * h[m - k] for k in range(1, m + 1))
        return Jet(h)

    def sqrt(self) -> "Jet":
        g, n = self.d, self.order
        h = np.zeros_like(g)
        h[0] = np.sqrt(g[0])
        for m in range(1, n + 1):
            s = sum(comb(m, k) * h[k] * h[m - k] for k in range(1, m))
            h[m] = (g[m] - s) / (2.0 * h[0])
        return Jet(h)

    def norm(self) -> "Jet":
        return self.dot(self).sqrt()

    def normalized(self) -> tuple["Jet", "Jet"]:
        """(unit vector jet, norm jet)."""
        n = self.norm()
        return self * n.reciprocal(), n

    def __truediv__(self, other):
        if isinstance(other, Jet):
            return self * other.reciprocal()
        return Jet(self.d / np.asarray(other, float))


def columns_to_matrix(c1: Jet, c2: Jet, c3: Jet) -> Jet:
    """Matrix jet with the given column jets."""
    n = min(c1.order, c2.order, c3.order)
    return Jet(np.stack([c1.d[: n + 1], c2.d[: n + 1], c3.d[: n + 1]], axis=-1))


def vee(S: np.ndarray) -> np.ndarray:
    """so(3) -> R^3, using the skew-symmetric part of S."""
    A = 0.5 * (S - S.T)
    return np.array([A[2, 1], A[0, 2], A[1, 0]])
