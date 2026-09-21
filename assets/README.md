# Stack assets

Brand assets for the whole Vayu stack, kept here rather than split across the
product repositories.

`logos/` holds unadopted *concept* SVGs for three products — Vayu, NavLink and
Navigator — with `logos/index.html` as a side-by-side preview. Comparing them
as a set is the artifact's entire purpose, so splitting them by repo would
destroy the thing while tidying it.

This repository is the stack's entry point (see the top-level README), which is
why whole-stack material lives here even though the code here is firmware. If a
logo is ever actually adopted, the chosen one belongs in its own product's
repository and this concept set has done its job.

Nothing in any build references these files.
