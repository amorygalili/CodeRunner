# Controls Engineering in the FIRST Robotics Competition

by **Tyler Veness**

This folder is a Markdown conversion of the book, one file per chapter. The
text, equations, and code come from the book's LaTeX source (commit `46d4bd5`,
2026-09-26); the figures are cropped from the published PDF.

Equations use `$...$` / `$$...$$` math, which renders on GitHub, in the
VS Code / VSCodium Markdown preview, and in Docusaurus with `remark-math`.

## Contents

- [Preface](preface.md)
- [0. Notes to the reader](00-notes-to-the-reader.md)

### Part I: Fundamentals of control theory

- [1. Control system basics](01-control-system-basics.md)
- [2. PID controllers](02-pid-controllers.md)
- [3. Application advice](03-application-advice.md)
- [4. Calculus](04-calculus.md)

### Part II: Modern control theory

- [5. Linear algebra](05-linear-algebra.md)
- [6. Continuous state-space control](06-continuous-state-space-control.md)
- [7. Discrete state-space control](07-discrete-state-space-control.md)
- [8. Nonlinear control](08-nonlinear-control.md)

### Part III: Estimation and localization

- [9. Stochastic control theory](09-stochastic-control-theory.md)
- [10. Pose estimation](10-pose-estimation.md)

### Part IV: System modeling

- [11. Dynamics](11-dynamics.md)
- [12. Newtonian mechanics examples](12-newtonian-mechanics-examples.md)
- [13. Lagrangian mechanics examples](13-lagrangian-mechanics-examples.md)
- [14. System identification](14-system-identification.md)

### Part V: Motion planning

- [15. Motion profiles](15-motion-profiles.md)
- [16. Configuration spaces](16-configuration-spaces.md)
- [17. Trajectory optimization](17-trajectory-optimization.md)

### Appendices

- [Appendix A. Simplifying block diagrams](A-simplifying-block-diagrams.md)
- [Appendix B. Linear-quadratic regulator](B-linear-quadratic-regulator.md)
- [Appendix C. Feedforwards](C-feedforwards.md)
- [Appendix D. Derivations](D-derivations.md)
- [Appendix E. Classical control theory](E-classical-control-theory.md)

### Back matter

- [Glossary](glossary.md)

## License and attribution

*Controls Engineering in the FIRST Robotics Competition* © 2017-2026 Tyler Veness,
<https://github.com/calcmogul/controls-engineering-in-frc>. The latest PDF is at
<https://file.tavsys.net/control/controls-engineering-in-frc.pdf>.

The book is licensed under
[CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/); this
Markdown adaptation is shared under the same license. The code snippets are
released by the author under the
[3-clause BSD license](https://github.com/calcmogul/controls-engineering-in-frc/blob/main/LICENSE.BSD).

**Changes from the original:** converted from LaTeX to Markdown and split into
one file per chapter; figures are raster crops of the PDF; the index and
chapter photos are omitted; glossary terms are plain text rather than links;
citations appear as footnotes. Numbering of chapters, sections, figures,
tables, theorems, and referenced equations matches the PDF.
