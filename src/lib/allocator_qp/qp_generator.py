import osqp
import numpy as np
import scipy as sp
from scipy import sparse

G = sparse.random(4,4,1.0)

H = np.diag([1,1,0.1, 0.5])

P = sparse.csc_matrix(G.T @ H @ G)

mu_ref = np.array([1,2,3,4])

q = G.T @ H @ mu_ref 

A = sparse.csc_matrix(np.eye(4))
l = np.zeros(4)
u = 10.0 * np.ones(4)

prob = osqp.OSQP()
prob.setup(P, q, A, l, u)

prob.codegen('allocator_qp', parameters='matrices', project_type='Makefile')
