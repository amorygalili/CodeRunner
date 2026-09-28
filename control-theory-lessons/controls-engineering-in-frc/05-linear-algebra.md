# Chapter 5: Linear algebra

Modern control theory borrows concepts from linear algebra. At first, linear algebra may appear very abstract, but there are simple geometric intuitions underlying it. We’ll be linking to 3Blue1Brown’s [*Essence of linear algebra*](https://www.3blue1brown.com/?topic=linear-algebra) video series because it’s better at conveying that intuition than static text.

> **Video:** [“Essence of linear algebra preview” (5 minutes)](https://www.3blue1brown.com/lessons/eola-preview/) — 3Blue1Brown

## 5.1 Vectors

> **Video:** [“Vectors, what even are they?” (5 minutes)](https://www.3blue1brown.com/lessons/vectors/) — 3Blue1Brown

## 5.2 Linear combinations, span, and basis vectors

> **Video:** [“Linear combination, span, and basis vectors” (10 minutes)](https://www.3blue1brown.com/lessons/span/) — 3Blue1Brown

## 5.3 Linear transformations and matrices

> **Video:** [“Linear transformations and matrices” (11 minutes)](https://www.3blue1brown.com/lessons/linear-transformations/) — 3Blue1Brown

## 5.4 Matrix multiplication

> **Video:** [“Matrix multiplication as composition” (10 minutes)](https://www.3blue1brown.com/lessons/matrix-multiplication/) — 3Blue1Brown

## 5.5 The determinant

> **Video:** [“The determinant” (10 minutes)](https://www.3blue1brown.com/lessons/determinant/) — 3Blue1Brown

## 5.6 Inverse matrices, column space, and null space

> **Video:** [“Inverse matrices, column space, and null space” (12 minutes)](https://www.3blue1brown.com/lessons/inverse-matrices/) — 3Blue1Brown

## 5.7 Nonsquare matrices

> **Video:** [“Nonsquare matrices as transformations between dimensions” (4 minutes)](https://www.3blue1brown.com/lessons/nonsquare-matrices/) — 3Blue1Brown

## 5.8 Eigenvectors and eigenvalues

> **Video:** [“Eigenvectors and eigenvalues” (17 minutes)](https://www.3blue1brown.com/lessons/eigenvalues/) — 3Blue1Brown

## 5.9 Basic notation

The dimensionality of a matrix is specified by a row-column pair. For example, a matrix with two rows and three columns is a two-by-three matrix. Matrices are named with uppercase letters while vectors are named with lowercase letters. An optional subscript ${m \times n}$ denotes the matrix having $m$ rows and $n$ columns.

$$
\mathbf{x} =
  \begin{bmatrix}
    1 \\
    2
  \end{bmatrix}
  \quad
  \mathbf{A}_{2 \times 3} =
  \begin{bmatrix}
    1 & 2 & 3 \\
    4 & 5 & 6
  \end{bmatrix}
$$

## 5.10 Matrix operators

### 5.10.1 Transpose

$\mathbf{A}^{\mathsf{T}}$ denotes the transpose of $\mathbf{A}$, which flips the matrix across its diagonal such that the rows become columns and vice versa.

### 5.10.2 Inverse

$\mathbf{A}^{-1}$ denotes the inverse of $\mathbf{A}$, which is defined as the matrix such that $\mathbf{A}^{-1}\mathbf{A} = \mathbf{A}\mathbf{A}^{-1} = \mathbf{I}$.

### 5.10.3 Inverse transpose

$\mathbf{A}^{-\mathsf{T}}$ denotes the inverse transpose of $\mathbf{A}$, which performs an inverse then a transpose.

### 5.10.4 Pseudoinverse

$\mathbf{A}^+$ denotes the Moore-Penrose pseudoinverse of $\mathbf{A}$, which is a generalization of the inverse matrix to nonsquare matrices. The pseudoinverse is an approximate inverse in the least-squares sense, so it can solve least-squares problems of the form $\mathbf{A}\mathbf{x} = \mathbf{b}$ via $\mathbf{x} = \mathbf{A}^+\mathbf{b}$. See table 5.1 for pseudoinverse definitions by system type.

*Table 5.1: Pseudoinverses by system type.*

| **System type** | **Shape of A** | **Name of** $\mathbf{A}^+$ | **Value of** $\mathbf{A}^+$ |
|:--:|:--:|:--:|:--:|
| Well-determined | Square | Inverse | $\mathbf{A}^{-1}$ |
| Overdetermined | Tall | Left pseudoinverse | $(\mathbf{A}^{\mathsf{T}}\mathbf{A})^{-1}\mathbf{A}^{\mathsf{T}}$ |
| Underdetermined | Wide | Right pseudoinverse | $\mathbf{A}^{\mathsf{T}}(\mathbf{A}\mathbf{A}^{\mathsf{T}})^{-1}$ |

### 5.10.5 Diagonal

Let $\mathbf{x}$ be a vector and $\mathbf{A}$ and $\mathbf{B}$ be matrices.

$\operatorname{diag}(\mathbf{x})$ creates a diagonal matrix with the elements of $\mathbf{x}$ along the diagonal.

$$
\operatorname{diag}(\mathbf{x}) = \operatorname{diag}(x_1,\, \ldots,\, x_n) =
  \begin{bmatrix}
    x_1 & 0 & \cdots & 0 \\
    0 & x_2 & & \vdots \\
    \vdots & & \ddots & 0 \\
    0 & \cdots & 0 & x_n
  \end{bmatrix}
$$

$\operatorname{diag}(\mathbf{A}, \mathbf{B})$ creates a block diagonal matrix.

$$
\operatorname{diag}(\mathbf{A}, \mathbf{B}) =
  \begin{bmatrix}
    \mathbf{A} & \mathbf{0} \\
    \mathbf{0} & \mathbf{B}
  \end{bmatrix}
$$

Operations on the $\operatorname{diag}()$ argument are applied element-wise.

$$
\operatorname{diag}\left(\frac{1}{\mathbf{x}^2}\right) =
  \begin{bmatrix}
    \frac{1}{x_1^2} & 0 & \cdots & 0 \\
    0 & \frac{1}{x_2^2} & & \vdots \\
    \vdots & & \ddots & 0 \\
    0 & \cdots & 0 & \frac{1}{x_n^2}
  \end{bmatrix}
$$

### 5.10.6 Trace

$\operatorname{tr}(\mathbf{A})$ denotes the trace of the square matrix $\mathbf{A}$, which is defined as the sum of the elements on the main diagonal.

## 5.11 Matrix types

### 5.11.1 By shape

A *row vector* is a matrix with one row.

$$
\begin{bmatrix}
    1 & 2
  \end{bmatrix}
$$

A *column vector* is a matrix with one column.

$$
\begin{bmatrix}
    1 \\
    2
  \end{bmatrix}
$$

A *square matrix* has the same number of rows and columns.

$$
\begin{bmatrix}
    1 & 2 \\
    3 & 4
  \end{bmatrix}
$$

A *rectangular matrix* has a different number of rows and columns.

$$
\begin{bmatrix}
    1 & 2 & 3 \\
    4 & 5 & 6
  \end{bmatrix}
$$

### 5.11.2 By contents

A *zero matrix* is a matrix filled with zeroes, denoted by $\mathbf{0}$.

$$
\mathbf{0}_{2 \times 3} =
  \begin{bmatrix}
    0 & 0 & 0 \\
    0 & 0 & 0
  \end{bmatrix}
$$

A *one vector* is a column vector filled with ones, denoted by $\mathbf{e}$.

$$
\mathbf{e} =
  \begin{bmatrix}
    1 \\
    1
  \end{bmatrix}
$$

A *one matrix* is a matrix filled with ones, denoted by $\mathbf{1}$.

$$
\mathbf{1}_{2 \times 3} =
  \begin{bmatrix}
    1 & 1 & 1 \\
    1 & 1 & 1
  \end{bmatrix}
$$

An *identity matrix* is a diagonal matrix of ones, denoted by $\mathbf{I}$.

$$
\mathbf{I}_{3 \times 3} =
  \begin{bmatrix}
    1 & 0 & 0 \\
    0 & 1 & 0 \\
    0 & 0 & 1
  \end{bmatrix}
  \quad
  \mathbf{I}_{2 \times 3} =
  \begin{bmatrix}
    1 & 0 & 0 \\
    0 & 1 & 0
  \end{bmatrix}
$$

A matrix’s *diagonal* or *main diagonal* is the list of entries with the same row and column index.

$$
\begin{bmatrix}
    {\color{red}1} & 0 & 0 \\
    0 & {\color{red}2} & 0 \\
    0 & 0 & {\color{red}3}
  \end{bmatrix}
$$

A matrix’s *antidiagonal* is the list of entries from the top-right to the bottom-left perpendicular to the diagonal.

$$
\begin{bmatrix}
    0 & 0 & {\color{red}1} \\
    0 & {\color{red}2} & 0 \\
    {\color{red}3} & 0 & 0
  \end{bmatrix}
$$

A *diagonal matrix* has nonzeroes restricted to the diagonal.

$$
\begin{bmatrix}
    1 & 0 & 0 \\
    0 & 2 & 0 \\
    0 & 0 & 3
  \end{bmatrix}
$$

A *block diagonal matrix* has matrices along its diagonal and zeroes elsewhere.

$$
\begin{bmatrix}
    1 & 2 & 0 & 0 & 0 \\
    3 & 4 & 0 & 0 & 0 \\
    0 & 0 & 1 & 2 & 3 \\
    0 & 0 & 4 & 5 & 6 \\
    0 & 0 & 7 & 8 & 9
  \end{bmatrix}
$$

A matrix’s *superdiagonal* is the set of entries above and to the right of the main diagonal.

$$
\begin{bmatrix}
    0 & {\color{red}1} & 0 \\
    0 & 0 & {\color{red}2} \\
    0 & 0 & 0
  \end{bmatrix}
$$

A matrix’s *subdiagonal* is the set of entries below and to the left of the main diagonal.

$$
\begin{bmatrix}
    0 & 0 & 0 \\
    {\color{red}1} & 0 & 0 \\
    0 & {\color{red}2} & 0
  \end{bmatrix}
$$

A *banded matrix* has nonzero elements restricted to a diagonal band. A *tridiagonal matrix* has nonzero elements restricted to the main diagonal, superdiagonal, and subdiagonal.

$$
\begin{bmatrix}
    2 & 1 & 0 & 0 \\
    3 & 2 & 1 & 0 \\
    0 & 3 & 2 & 1 \\
    0 & 0 & 3 & 2
  \end{bmatrix}
$$

### 5.11.3 By structure

A *symmetric matrix* is equal to its transpose.

$$
\begin{bmatrix}
    1 & 2 \\
    2 & 3
  \end{bmatrix}
$$

An *orthogonal matrix* $\mathbf{Q}$ has the following properties:

- $\mathbf{Q}^{\mathsf{T}}\mathbf{Q} = \mathbf{Q}\mathbf{Q}^{\mathsf{T}} = \mathbf{I}$

- $\mathbf{Q}^{\mathsf{T}} = \mathbf{Q}^{-1}$

- $\det(\mathbf{Q}) = 1 \text{ or } -1$

A *special orthogonal matrix* is an orthogonal matrix with a determinant of $1$.

### 5.11.4 Matrix definiteness

Tables 5.2 and 5.3 describe the different kinds of matrix definiteness.

*Table 5.2: Eigenvalue distribution and notation for each type of matrix definiteness. Let $\mathbf{M}$ be a matrix.*

|       **Type**        |    **Eigenvalues**    |         **Notation**         |
|:---------------------:|:---------------------:|:----------------------------:|
|   Negative definite   |   All $\lambda < 0$   |  $\mathbf{M} < \mathbf{0}$   |
| Negative semidefinite | All $\lambda \leq 0$  | $\mathbf{M} \leq \mathbf{0}$ |
| Positive semidefinite | All $\lambda \geq 0$  | $\mathbf{M} \geq \mathbf{0}$ |
|   Positive definite   |   All $\lambda > 0$   |  $\mathbf{M} > \mathbf{0}$   |
|      Indefinite       | Positive and negative |             N/A              |

*Table 5.3: Rigorous definition for each type of matrix definiteness. Let $\mathbf{M}$ be a matrix and let $\mathbf{x}$ and $\mathbf{y}$ be nonzero vectors.*

| **Type** | **Definition** |
|:--:|:--:|
| Negative definite | $\mathbf{x}^{\mathsf{T}}\mathbf{M}\mathbf{x} < 0$ for all $\mathbf{x}$ |
| Negative semidefinite | $\mathbf{x}^{\mathsf{T}}\mathbf{M}\mathbf{x} \leq 0$ for all $\mathbf{x}$ |
| Positive semidefinite | $\mathbf{x}^{\mathsf{T}}\mathbf{M}\mathbf{x} \geq 0$ for all $\mathbf{x}$ |
| Positive definite | $\mathbf{x}^{\mathsf{T}}\mathbf{M}\mathbf{x} > 0$ for all $\mathbf{x}$ |
| Indefinite | $\exists \mathbf{x}, \mathbf{y} \ni \mathbf{x}^{\mathsf{T}}\mathbf{M}\mathbf{x} < 0 < \mathbf{y}^{\mathsf{T}}\mathbf{M}\mathbf{y}$ |

## 5.12 Dense matrices

A dense matrix is filled with mostly nonzeroes. Algorithms designed for dense matrices can use Single Instruction, Multiple Data (SIMD) instructions to perform parts of a computation in parallel.

### 5.12.1 Solving dense linear systems

Solving linear systems with a decomposition followed by a triangular solve is more efficient and numerically stable than explicitly forming the inverse of the system matrix. Which decomposition to use depends on the system’s properties. In general, use:

- LLT for symmetric positive definite systems

- LDLT for symmetric positive semidefinite systems

- LU for square invertible systems

- QR for least-squares solving

See <https://libeigen.gitlab.io/eigen/docs-nightly/group__TutorialLinearAlgebra.html> for more specific guidance.

*Pivoting* is the act of permuting rows and columns during the solve. Column pivoting and full pivoting (permuting rows *and* columns) improve numerical stability at the cost of performance.

## 5.13 Sparse matrices

A sparse matrix is filled with mostly zeroes. Algorithms designed for sparse matrices can avoid work by skipping operations on zeroes.

### 5.13.1 Storage

Sparse matrices save space and compute by only storing and operating on the nonzero elements of the matrix. There are several possible storage schemes[^1], but Compressed Sparse Column format is popular.

### 5.13.2 Sparsity pattern

The *sparsity pattern* of a matrix is its pattern of structural nonzero elements, which represents input-output coupling. An element is *structurally nonzero* if coupling exists (i.e., it *can* be nonzero), and it’s *structurally zero* otherwise.

*Banded sparsity* is common in trajectory optimization problems.

$$
\begin{bmatrix}
    \times & \times & \cdot & \cdot & \cdot & \cdot \\
    \times & \times & \cdot & \cdot & \cdot & \cdot \\
    \cdot & \cdot & \times & \times & \cdot & \cdot \\
    \cdot & \cdot & \times & \times & \cdot & \cdot \\
    \cdot & \cdot & \cdot & \cdot & \times & \times \\
    \cdot & \cdot & \cdot & \cdot & \times & \times
  \end{bmatrix}
$$

*Blocked sparsity* is common in finite element analysis problems.

$$
\begin{bmatrix}
    \times & \times & \cdot & \cdot & \times & \times \\
    \times & \times & \cdot & \cdot & \times & \times \\
    \cdot & \cdot & \cdot & \cdot & \cdot & \cdot \\
    \cdot & \cdot & \cdot & \cdot & \cdot & \cdot \\
    \times & \times & \cdot & \cdot & \cdot & \cdot \\
    \times & \times & \cdot & \cdot & \cdot & \cdot
  \end{bmatrix}
$$

When solving these two kinds of problems via iterative algorithms, the contents of the matrix often change but the sparsity pattern is fixed, even if an element happened to be zero for the current iteration.

> **Remark.** It can be beneficial for performance to keep nonstructural zeroes in the matrix storage if it maintains the same sparsity pattern between iterations.

### 5.13.3 Solving sparse linear systems

Solving linear systems with a decomposition followed by a triangular solve is more efficient and numerically stable than explicitly forming the inverse of the system matrix. Which decomposition to use depends on the system’s properties. In general, use:

- LLT for symmetric positive definite systems

- simplicial LDLT for symmetric positive semidefinite systems with a banded sparsity pattern

- supernodal LDLT for symmetric positive semidefinite systems with a blocked sparsity pattern

- LU for square invertible systems

- QR for least-squares solving

See <https://libeigen.gitlab.io/eigen/docs-nightly/group__TopicSparseSystems.html> for more specific guidance.

*Pivoting* is the act of permuting rows and columns during the solve. Factorizing a sparse matrix can result in *fill-in*, where the factorization has more nonzeros than the original matrix. For sparse matrices, we typically want a fill-in reducing permutation.

Sparse system solvers typically have two steps: a symbolic factorization that determines the sparsity pattern and finds a fill-in reducing permutation, and a numerical factorization step that operates on the specific matrix. Keeping the sparsity pattern the same between solves allows reusing the result of the first step.

## 5.14 Common control theory matrix equations

Here’s some common matrix equations from control theory we’ll use later on. Solvers for them exist in [Drake](https://github.com/RobotLocomotion/drake) (C++) and [SciPy](https://github.com/scipy/scipy) (Python).

### 5.14.1 Continuous algebraic Riccati equation (CARE)

The continuous algebraic Riccati equation (CARE) appears in the solution to the continuous time LQ problem.

$$
\mathbf{A}^{\mathsf{T}}\mathbf{X} + \mathbf{X}\mathbf{A} -
    \mathbf{X}\mathbf{B}\mathbf{R}^{-1}\mathbf{B}^{\mathsf{T}}\mathbf{X} + \mathbf{Q} = \mathbf{0}
$$

### 5.14.2 Discrete algebraic Riccati equation (DARE)

The discrete algebraic Riccati equation (DARE) appears in the solution to the discrete time LQ problem.

$$
\mathbf{X} = \mathbf{A}^{\mathsf{T}}\mathbf{X}\mathbf{A} - (\mathbf{A}^{\mathsf{T}}\mathbf{X}\mathbf{B})(\mathbf{R} +
    \mathbf{B}^{\mathsf{T}}\mathbf{X}\mathbf{B})^{-1} \mathbf{B}^{\mathsf{T}}\mathbf{X}\mathbf{A} + \mathbf{Q}
$$

Snippets 5.1 and 5.2 compute the unique stabilizing solution to the discrete algebraic Riccati equation. A robust implementation should also enforce the following preconditions:

1.  $\mathbf{Q} = \mathbf{Q}^{\mathsf{T}} \geq \mathbf{0}$ and $\mathbf{R} = \mathbf{R}^{\mathsf{T}} > \mathbf{0}$.

2.  $(\mathbf{A}, \mathbf{B})$ is a stabilizable pair (see subsection [6.12.4](06-continuous-state-space-control.md#6124-stabilizability)).

3.  $(\mathbf{A}, \mathbf{C})$ is a detectable pair where $\mathbf{Q} = \mathbf{C}^{\mathsf{T}}\mathbf{C}$ (see section [6.12.8](06-continuous-state-space-control.md#6128-detectability)).

```cpp
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>

template <int States, int Inputs>
Eigen::Matrix<double, States, States> dare(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, Inputs>& B,
    const Eigen::Matrix<double, States, States>& Q,
    const Eigen::Matrix<double, Inputs, Inputs>& R) {
  // [1] E. K.-W. Chu, H.-Y. Fan, W.-W. Lin & C.-S. Wang
  //     "Structure-Preserving Algorithms for Periodic Discrete-Time
  //     Algebraic Riccati Equations",
  //     International Journal of Control, 77:8, 767-788, 2004.
  //     DOI: 10.1080/00207170410001714988
  //
  // Implements SDA algorithm on p. 5 of [1] (initial A, G, H are from (4)).
  using StateMatrix = Eigen::Matrix<double, States, States>;

  StateMatrix A_k = A;
  StateMatrix G_k = B * R.llt().solve(B.transpose());
  StateMatrix H_k;
  StateMatrix H_k1 = Q;

  do {
    H_k = H_k1;

    StateMatrix W = StateMatrix::Identity() + G_k * H_k;

    auto W_solver = W.lu();
    StateMatrix V_1 = W_solver.solve(A_k);
    StateMatrix V_2 = W_solver.solve(G_k.transpose()).transpose();

    G_k += A_k * V_2 * A_k.transpose();
    H_k1 = H_k + V_1.transpose() * H_k * A_k;
    A_k *= V_1;
  } while ((H_k1 - H_k).norm() > 1e-10 * H_k1.norm());

  return H_k1;
}
```

*Snippet 5.1. Dense discrete algebraic Riccati equation solver in C++*

```cpp
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>

Eigen::SparseMatrix<double> identity(int rows) {
  return Eigen::SparseMatrix<double>{
      Eigen::VectorXd::Constant(rows, 1.0).asDiagonal()};
}

Eigen::SparseMatrix<double> dare(const Eigen::SparseMatrix<double>& A,
                                 const Eigen::SparseMatrix<double>& B,
                                 const Eigen::SparseMatrix<double>& Q,
                                 const Eigen::SparseMatrix<double>& R) {
  // [1] E. K.-W. Chu, H.-Y. Fan, W.-W. Lin & C.-S. Wang
  //     "Structure-Preserving Algorithms for Periodic Discrete-Time
  //     Algebraic Riccati Equations",
  //     International Journal of Control, 77:8, 767-788, 2004.
  //     DOI: 10.1080/00207170410001714988
  //
  // Implements SDA algorithm on p. 5 of [1] (initial A, G, H are from (4)).
  using StateMatrix = Eigen::SparseMatrix<double>;

  StateMatrix A_k = A;
  Eigen::SimplicialLLT R_llt{R};
  StateMatrix G_k = B * R_llt.solve(StateMatrix{B.transpose()});
  StateMatrix H_k;
  StateMatrix H_k1 = Q;

  do {
    H_k = H_k1;

    StateMatrix W = identity(A.rows()) + G_k * H_k;

    Eigen::SparseLU W_solver{W};
    StateMatrix V_1 = W_solver.solve(A_k);
    StateMatrix V_2 = W_solver.solve(G_k);

    G_k += StateMatrix{A_k * V_2 * A_k.transpose()};
    H_k1 = H_k + V_1.transpose() * H_k * A_k;
    A_k = A_k * StateMatrix{V_1};
  } while ((H_k1 - H_k).norm() > 1e-10 * H_k1.norm());

  return H_k1;
}
```

*Snippet 5.2. Sparse discrete algebraic Riccati equation solver in C++*

### 5.14.3 Continuous Lyapunov equation

The continuous Lyapunov equation appears in controllability/observability analysis of continuous time systems.

$$
\mathbf{A}\mathbf{X} + \mathbf{X}\mathbf{A}^{\mathsf{T}} + \mathbf{Q} = \mathbf{0}
$$

### 5.14.4 Discrete Lyapunov equation

The discrete Lyapunov equation appears in controllability/observability analysis of discrete time systems.

$$
\mathbf{A}\mathbf{X}\mathbf{A}^{\mathsf{T}} - \mathbf{X} + \mathbf{Q} = \mathbf{0}
$$

## 5.15 Matrix calculus

Matrix calculus uses partial derivatives. See subsection [4.1.4](04-calculus.md#414-partial-derivatives) for how they work.

We’ll need a vector-valued function to demonstrate some common operations in matrix calculus. Let $\mathbf{f}(\mathbf{x}, \mathbf{u})$ be a vector-valued function defined as

$$
\mathbf{f}(\mathbf{x}, \mathbf{u}) =
  \begin{bmatrix}
    f_1(\mathbf{x}, \mathbf{u}) \\
    \vdots \\
    f_m(\mathbf{x}, \mathbf{u})
  \end{bmatrix}
  \text{where }
  \mathbf{x} =
  \begin{bmatrix}
    x_1 \\
    \vdots \\
    x_m
  \end{bmatrix}
  \text{and }
  \mathbf{u} =
  \begin{bmatrix}
    u_1 \\
    \vdots \\
    u_n
  \end{bmatrix}
$$

### 5.15.1 Jacobian

The Jacobian is the first-order partial derivative of a vector-valued function with respect to one of its vector arguments. The columns of the Jacobian of $\mathbf{f}$ are filled with partial derivatives of $\mathbf{f}$’s rows with respect to each of the argument’s elements. For example, the Jacobian of $\mathbf{f}$ with respect to $\mathbf{x}$ is

$$
\frac{\partial \mathbf{f}(\mathbf{x}, \mathbf{u})}{\partial \mathbf{x}} =
  \begin{bmatrix}
    \frac{\partial \mathbf{f}(\mathbf{x}, \mathbf{u})}{\partial x_1} & \ldots &
      \frac{\partial \mathbf{f}(\mathbf{x}, \mathbf{u})}{\partial x_m}
  \end{bmatrix} =
  \begin{bmatrix}
    \frac{\partial f_1}{\partial x_1} & \ldots &
      \frac{\partial f_1}{\partial x_m} \\
    \vdots & \ddots & \vdots \\
    \frac{\partial f_m}{\partial x_1} & \ldots &
      \frac{\partial f_m}{\partial x_m}
  \end{bmatrix}
$$

$\frac{\partial f_1}{\partial x_1}$ is the partial derivative of the first row of $\mathbf{f}$ with respect to the first row of $\mathbf{x}$, and so on for all rows of $\mathbf{f}$ and $\mathbf{x}$. This has $m^2$ permutations and thus produces a square matrix.

The Jacobian of $\mathbf{f}$ with respect to $\mathbf{u}$ is

$$
\frac{\partial \mathbf{f}(\mathbf{x}, \mathbf{u})}{\partial \mathbf{u}} =
  \begin{bmatrix}
    \frac{\partial \mathbf{f}(\mathbf{x}, \mathbf{u})}{\partial u_1} & \ldots &
      \frac{\partial \mathbf{f}(\mathbf{x}, \mathbf{u})}{\partial u_n}
  \end{bmatrix} =
  \begin{bmatrix}
    \frac{\partial f_1}{\partial u_1} & \ldots &
      \frac{\partial f_1}{\partial u_n} \\
    \vdots & \ddots & \vdots \\
    \frac{\partial f_m}{\partial u_1} & \ldots &
      \frac{\partial f_m}{\partial u_n}
  \end{bmatrix}
$$

$\frac{\partial f_1}{\partial u_1}$ is the partial derivative of the first row of $\mathbf{f}$ with respect to the first row of $\mathbf{u}$, and so on for all rows of $\mathbf{f}$ and $\mathbf{u}$. This has $m \times n$ permutations and can produce a nonsquare matrix if $m \neq n$.

### 5.15.2 Hessian

The Hessian is the second-order partial derivative of a vector-valued function with respect to one of its vector arguments. For example, the Hessian of $\mathbf{f}$ with respect to $\mathbf{x}$ is

$$
\frac{\partial^2 \mathbf{f}(\mathbf{x}, \mathbf{u})}{\partial \mathbf{x}^2} =
  \begin{bmatrix}
    \frac{\partial^2 \mathbf{f}(\mathbf{x}, \mathbf{u})}{\partial x_1^2} & \ldots &
      \frac{\partial^2 \mathbf{f}(\mathbf{x}, \mathbf{u})}{\partial x_m^2}
  \end{bmatrix} =
  \begin{bmatrix}
    \frac{\partial^2 f_1}{\partial x_1^2} & \ldots &
      \frac{\partial^2 f_1}{\partial x_m^2} \\
    \vdots & \ddots & \vdots \\
    \frac{\partial^2 f_m}{\partial x_1^2} & \ldots &
      \frac{\partial^2 f_m}{\partial x_m^2}
  \end{bmatrix}
$$

and the Hessian of $\mathbf{f}$ with respect to $\mathbf{u}$ is

$$
\frac{\partial^2 \mathbf{f}(\mathbf{x}, \mathbf{u})}{\partial \mathbf{u}^2} =
  \begin{bmatrix}
    \frac{\partial^2 \mathbf{f}(\mathbf{x}, \mathbf{u})}{\partial u_1^2} & \ldots &
      \frac{\partial^2 \mathbf{f}(\mathbf{x}, \mathbf{u})}{\partial u_n^2}
  \end{bmatrix} =
  \begin{bmatrix}
    \frac{\partial^2 f_1}{\partial u_1^2} & \ldots &
      \frac{\partial^2 f_1}{\partial u_n^2} \\
    \vdots & \ddots & \vdots \\
    \frac{\partial^2 f_m}{\partial u_1^2} & \ldots &
      \frac{\partial^2 f_m}{\partial u_n^2}
  \end{bmatrix}
$$

### 5.15.3 Useful identities

Here’s some useful matrix calculus identities pulled from Wikipedia’s table.[^2]

> **Theorem 5.15.1.**
>
> $\frac{\partial \mathbf{x}^{\mathsf{T}}\mathbf{A}\mathbf{x}}{\partial\mathbf{x}} = 2\mathbf{A}\mathbf{x}$ where $\mathbf{A}$ is symmetric.

> **Theorem 5.15.2.**
>
> $\frac{\partial (\mathbf{A}\mathbf{x} + \mathbf{b})^{\mathsf{T}}\mathbf{C} (\mathbf{D}\mathbf{x} + \mathbf{e})}{\partial\mathbf{x}} = \mathbf{A}^{\mathsf{T}}\mathbf{C}(\mathbf{D}\mathbf{x} + \mathbf{e}) + \mathbf{D}^{\mathsf{T}}\mathbf{C}^{\mathsf{T}} (\mathbf{A}\mathbf{x} + \mathbf{b})$

> **Corollary 5.15.3.**
>
> $\frac{\partial (\mathbf{A}\mathbf{x} + \mathbf{b})^{\mathsf{T}}\mathbf{C} (\mathbf{A}\mathbf{x} + \mathbf{b})}{\partial\mathbf{x}} = 2\mathbf{A}^{\mathsf{T}}\mathbf{C}(\mathbf{A}\mathbf{x} + \mathbf{b})$ where $\mathbf{C}$ is symmetric.
>
> Proof:
>
> $$
> \begin{aligned}
> \frac{\partial (\mathbf{A}\mathbf{x} + \mathbf{b})^{\mathsf{T}}\mathbf{C}
>       (\mathbf{A}\mathbf{x} + \mathbf{b})}{\partial\mathbf{x}} &=
>       \mathbf{A}^{\mathsf{T}}\mathbf{C}(\mathbf{A}\mathbf{x} + \mathbf{b}) + \mathbf{A}^{\mathsf{T}}\mathbf{C}^{\mathsf{T}}
>       (\mathbf{A}\mathbf{x} + \mathbf{b}) \\
> \frac{\partial (\mathbf{A}\mathbf{x} + \mathbf{b})^{\mathsf{T}}\mathbf{C}
>       (\mathbf{A}\mathbf{x} + \mathbf{b})}{\partial\mathbf{x}} &=
>       (\mathbf{A}^{\mathsf{T}}\mathbf{C} + \mathbf{A}^{\mathsf{T}}\mathbf{C}^{\mathsf{T}})(\mathbf{A}\mathbf{x} + \mathbf{b})
> \end{aligned}
> $$
>
> $\mathbf{C}$ is symmetric, so
>
> $$
> \begin{aligned}
> \frac{\partial (\mathbf{A}\mathbf{x} + \mathbf{b})^{\mathsf{T}}\mathbf{C}
>       (\mathbf{A}\mathbf{x} + \mathbf{b})}{\partial\mathbf{x}} &=
>       (\mathbf{A}^{\mathsf{T}}\mathbf{C} + \mathbf{A}^{\mathsf{T}}\mathbf{C})(\mathbf{A}\mathbf{x} + \mathbf{b}) \\
> \frac{\partial (\mathbf{A}\mathbf{x} + \mathbf{b})^{\mathsf{T}}\mathbf{C}
>       (\mathbf{A}\mathbf{x} + \mathbf{b})}{\partial\mathbf{x}} &=
>       2\mathbf{A}^{\mathsf{T}}\mathbf{C}(\mathbf{A}\mathbf{x} + \mathbf{b})
> \end{aligned}
> $$
>

[^1]: <https://en.wikipedia.org/wiki/Sparse_matrix#Storage>

[^2]: <https://en.wikipedia.org/wiki/Matrix_calculus#Identities>
