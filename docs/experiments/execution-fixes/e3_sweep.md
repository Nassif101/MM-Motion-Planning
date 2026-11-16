# Fixed-margin sweep (E3 Task 4)

Safe: zero contacts and a true clearance never below 0.02 m in every run, reached or not. Clearance: the smallest over the runs, any robot part against any obstacle. Incomplete: a run without a summary, or a summary without a finite clearance and a contact count; such a cell is neither safe nor unsafe.

| Cell | Mode | Margin (m) | Reached | Runs with contact | Min clearance (mm) | Safe |
|---|---|---|---|---|---|---|
| e3_beam_straight_1 | ideal | 0.05 | 20/20 | 0 | 39.3 | yes |
| e3_beam_straight_1 | ideal | 0.04 | 20/20 | 0 | 3.7 | no |
| e3_beam_straight_1 | ideal | 0.03 | 20/20 | 0 | 19.0 | no |
| e3_beam_straight_1 | ideal | 0.02 | 20/20 | 0 | 8.9 | no |
| e3_beam_straight_1 | realistic | 0.05 | 20/20 | 0 | 31.3 | yes |
| e3_beam_straight_1 | realistic | 0.04 | 20/20 | 0 | 17.5 | no |
| e3_beam_straight_1 | realistic | 0.03 | 20/20 | 0 | 13.4 | no |
| e3_beam_straight_1 | realistic | 0.02 | 20/20 | 0 | 3.9 | no |
| e3_beam_straight_2 | ideal | 0.05 | 18/20 | 0 | 38.7 | yes |
| e3_beam_straight_2 | ideal | 0.04 | 20/20 | 0 | 27.2 | yes |
| e3_beam_straight_2 | ideal | 0.03 | 20/20 | 0 | 18.1 | no |
| e3_beam_straight_2 | ideal | 0.02 | 20/20 | 0 | 8.0 | no |
| e3_beam_straight_2 | realistic | 0.05 | 20/20 | 0 | 30.8 | yes |
| e3_beam_straight_2 | realistic | 0.04 | 20/20 | 0 | 20.2 | yes |
| e3_beam_straight_2 | realistic | 0.03 | 19/20 | 0 | 9.0 | no |
| e3_beam_straight_2 | realistic | 0.02 | 20/20 | 4 | -1.3 | no |
| e3_beam_straight_3 | ideal | 0.05 | 19/20 | 0 | 38.6 | yes |
| e3_beam_straight_3 | ideal | 0.04 | 18/20 | 0 | 29.3 | yes |
| e3_beam_straight_3 | ideal | 0.03 | 16/20 | 0 | 18.8 | no |
| e3_beam_straight_3 | ideal | 0.02 | 20/20 | 1 | -4.4 | no |
| e3_beam_straight_3 | realistic | 0.05 | 18/20 | 0 | 32.1 | yes |
| e3_beam_straight_3 | realistic | 0.04 | 20/20 | 0 | 20.2 | yes |
| e3_beam_straight_3 | realistic | 0.03 | 16/20 | 0 | 5.6 | no |
| e3_beam_straight_3 | realistic | 0.02 | 17/20 | 1 | -1.0 | no |
| e3_beam_straight_4 | ideal | 0.05 | 13/20 | 0 | 39.9 | yes |
| e3_beam_straight_4 | ideal | 0.04 | 18/20 | 0 | 29.7 | yes |
| e3_beam_straight_4 | ideal | 0.03 | 15/20 | 0 | 18.7 | no |
| e3_beam_straight_4 | ideal | 0.02 | 20/20 | 0 | 5.4 | no |
| e3_beam_straight_4 | realistic | 0.05 | 19/20 | 0 | 29.7 | yes |
| e3_beam_straight_4 | realistic | 0.04 | 18/20 | 0 | 24.4 | yes |
| e3_beam_straight_4 | realistic | 0.03 | 17/20 | 0 | 9.0 | no |
| e3_beam_straight_4 | realistic | 0.02 | 18/20 | 0 | 3.4 | no |
| e3_beam_turn_1 | ideal | 0.05 | 20/20 | 0 | 38.5 | yes |
| e3_beam_turn_1 | ideal | 0.04 | 20/20 | 0 | 28.7 | yes |
| e3_beam_turn_1 | ideal | 0.03 | 20/20 | 0 | 19.0 | no |
| e3_beam_turn_1 | ideal | 0.02 | 19/20 | 2 | -12.0 | no |
| e3_beam_turn_1 | realistic | 0.05 | 20/20 | 0 | 29.9 | yes |
| e3_beam_turn_1 | realistic | 0.04 | 19/20 | 0 | 19.2 | no |
| e3_beam_turn_1 | realistic | 0.03 | 20/20 | 0 | 8.2 | no |
| e3_beam_turn_1 | realistic | 0.02 | 18/20 | 4 | -8.2 | no |
| e3_beam_turn_2 | ideal | 0.05 | 19/20 | 0 | 39.0 | yes |
| e3_beam_turn_2 | ideal | 0.04 | 18/20 | 0 | 28.3 | yes |
| e3_beam_turn_2 | ideal | 0.03 | 17/20 | 0 | 18.7 | no |
| e3_beam_turn_2 | ideal | 0.02 | 19/20 | 2 | -3.7 | no |
| e3_beam_turn_2 | realistic | 0.05 | 20/20 | 0 | 30.4 | yes |
| e3_beam_turn_2 | realistic | 0.04 | 19/20 | 0 | 15.3 | no |
| e3_beam_turn_2 | realistic | 0.03 | 19/20 | 0 | 8.3 | no |
| e3_beam_turn_2 | realistic | 0.02 | 20/20 | 1 | -0.3 | no |
| e3_beam_turn_3 | ideal | 0.05 | 18/20 | 0 | 39.3 | yes |
| e3_beam_turn_3 | ideal | 0.04 | 19/20 | 0 | 28.2 | yes |
| e3_beam_turn_3 | ideal | 0.03 | 18/20 | 0 | 9.7 | no |
| e3_beam_turn_3 | ideal | 0.02 | 18/20 | 0 | 6.0 | no |
| e3_beam_turn_3 | realistic | 0.05 | 19/20 | 0 | 28.3 | yes |
| e3_beam_turn_3 | realistic | 0.04 | 17/20 | 0 | 18.1 | no |
| e3_beam_turn_3 | realistic | 0.03 | 18/20 | 0 | 6.3 | no |
| e3_beam_turn_3 | realistic | 0.02 | 19/20 | 0 | 0.6 | no |
| e3_beam_turn_4 | ideal | 0.05 | 18/20 | 0 | 40.3 | yes |
| e3_beam_turn_4 | ideal | 0.04 | 13/20 | 0 | 14.3 | no |
| e3_beam_turn_4 | ideal | 0.03 | 14/20 | 0 | 19.0 | no |
| e3_beam_turn_4 | ideal | 0.02 | 16/20 | 0 | 8.7 | no |
| e3_beam_turn_4 | realistic | 0.05 | 18/20 | 0 | 35.7 | yes |
| e3_beam_turn_4 | realistic | 0.04 | 18/20 | 0 | 16.4 | no |
| e3_beam_turn_4 | realistic | 0.03 | 16/20 | 0 | 9.8 | no |
| e3_beam_turn_4 | realistic | 0.02 | 13/20 | 0 | 0.5 | no |
| e3_gap_straight_1 | ideal | 0.05 | 18/20 | 0 | 45.2 | yes |
| e3_gap_straight_1 | ideal | 0.04 | 17/20 | 0 | 35.1 | yes |
| e3_gap_straight_1 | ideal | 0.03 | 16/20 | 0 | 23.0 | yes |
| e3_gap_straight_1 | ideal | 0.02 | 16/20 | 0 | 11.5 | no |
| e3_gap_straight_1 | realistic | 0.05 | 17/20 | 0 | 39.9 | yes |
| e3_gap_straight_1 | realistic | 0.04 | 20/20 | 0 | 30.8 | yes |
| e3_gap_straight_1 | realistic | 0.03 | 19/20 | 0 | 16.9 | no |
| e3_gap_straight_1 | realistic | 0.02 | 19/20 | 0 | 8.7 | no |
| e3_gap_straight_2 | ideal | 0.05 | 12/20 | 0 | 44.6 | yes |
| e3_gap_straight_2 | ideal | 0.04 | 14/20 | 0 | 34.7 | yes |
| e3_gap_straight_2 | ideal | 0.03 | 17/20 | 0 | 25.0 | yes |
| e3_gap_straight_2 | ideal | 0.02 | 12/20 | 0 | 15.0 | no |
| e3_gap_straight_2 | realistic | 0.05 | 17/20 | 0 | 40.4 | yes |
| e3_gap_straight_2 | realistic | 0.04 | 17/20 | 0 | 30.3 | yes |
| e3_gap_straight_2 | realistic | 0.03 | 17/20 | 0 | 11.9 | no |
| e3_gap_straight_2 | realistic | 0.02 | 16/20 | 0 | 8.5 | no |
| e3_gap_straight_3 | ideal | 0.05 | 7/20 | 0 | 42.5 | yes |
| e3_gap_straight_3 | ideal | 0.04 | 4/20 | 0 | 31.2 | yes |
| e3_gap_straight_3 | ideal | 0.03 | 10/20 | 0 | 22.8 | yes |
| e3_gap_straight_3 | ideal | 0.02 | 10/20 | 0 | 12.4 | no |
| e3_gap_straight_3 | realistic | 0.05 | 13/20 | 0 | 29.5 | yes |
| e3_gap_straight_3 | realistic | 0.04 | 15/20 | 0 | 29.8 | yes |
| e3_gap_straight_3 | realistic | 0.03 | 17/20 | 1 | -0.1 | no |
| e3_gap_straight_3 | realistic | 0.02 | 13/20 | 0 | 7.0 | no |
| e3_gap_straight_4 | ideal | 0.05 | 1/20 | 1 | -23.9 | no |
| e3_gap_straight_4 | ideal | 0.04 | 1/20 | 0 | 31.6 | yes |
| e3_gap_straight_4 | ideal | 0.03 | 3/20 | 0 | 21.8 | yes |
| e3_gap_straight_4 | ideal | 0.02 | 4/20 | 0 | 12.5 | no |
| e3_gap_straight_4 | realistic | 0.05 | 2/20 | 0 | 10.4 | no |
| e3_gap_straight_4 | realistic | 0.04 | 2/20 | 1 | -0.2 | no |
| e3_gap_straight_4 | realistic | 0.03 | 8/20 | 0 | 3.9 | no |
| e3_gap_straight_4 | realistic | 0.02 | 7/20 | 0 | 9.3 | no |
| e3_gap_turn_1 | ideal | 0.05 | 15/20 | 0 | 44.9 | yes |
| e3_gap_turn_1 | ideal | 0.04 | 15/20 | 0 | 34.2 | yes |
| e3_gap_turn_1 | ideal | 0.03 | 16/20 | 0 | 24.8 | yes |
| e3_gap_turn_1 | ideal | 0.02 | 8/20 | 0 | 14.4 | no |
| e3_gap_turn_1 | realistic | 0.05 | 17/20 | 0 | 34.9 | yes |
| e3_gap_turn_1 | realistic | 0.04 | 17/20 | 0 | 31.7 | yes |
| e3_gap_turn_1 | realistic | 0.03 | 14/20 | 0 | 18.0 | no |
| e3_gap_turn_1 | realistic | 0.02 | 16/20 | 0 | 8.5 | no |
| e3_gap_turn_2 | ideal | 0.05 | 11/20 | 0 | 43.6 | yes |
| e3_gap_turn_2 | ideal | 0.04 | 14/20 | 0 | 34.8 | yes |
| e3_gap_turn_2 | ideal | 0.03 | 13/20 | 0 | 22.6 | yes |
| e3_gap_turn_2 | ideal | 0.02 | 18/20 | 0 | 13.6 | no |
| e3_gap_turn_2 | realistic | 0.05 | 9/20 | 0 | 6.9 | no |
| e3_gap_turn_2 | realistic | 0.04 | 19/20 | 0 | 31.6 | yes |
| e3_gap_turn_2 | realistic | 0.03 | 13/20 | 0 | 19.0 | no |
| e3_gap_turn_2 | realistic | 0.02 | 13/20 | 0 | 2.6 | no |
| e3_gap_turn_3 | ideal | 0.05 | 5/20 | 0 | 43.6 | yes |
| e3_gap_turn_3 | ideal | 0.04 | 7/20 | 1 | -10.2 | no |
| e3_gap_turn_3 | ideal | 0.03 | 9/20 | 0 | 21.5 | yes |
| e3_gap_turn_3 | ideal | 0.02 | 14/20 | 0 | 13.2 | no |
| e3_gap_turn_3 | realistic | 0.05 | 8/20 | 0 | 10.0 | no |
| e3_gap_turn_3 | realistic | 0.04 | 14/20 | 0 | 20.5 | yes |
| e3_gap_turn_3 | realistic | 0.03 | 15/20 | 0 | 18.4 | no |
| e3_gap_turn_3 | realistic | 0.02 | 12/20 | 0 | 2.7 | no |
| e3_gap_turn_4 | ideal | 0.05 | 3/20 | 1 | -5.4 | no |
| e3_gap_turn_4 | ideal | 0.04 | 3/20 | 0 | 17.2 | no |
| e3_gap_turn_4 | ideal | 0.03 | 3/20 | 0 | 21.3 | yes |
| e3_gap_turn_4 | ideal | 0.02 | 5/20 | 0 | 10.8 | no |
| e3_gap_turn_4 | realistic | 0.05 | 3/20 | 1 | -0.3 | no |
| e3_gap_turn_4 | realistic | 0.04 | 2/20 | 0 | 8.0 | no |
| e3_gap_turn_4 | realistic | 0.03 | 3/20 | 2 | -1.6 | no |
| e3_gap_turn_4 | realistic | 0.02 | 1/20 | 1 | -1.5 | no |

Best fixed margin (realistic, safe on every cell of a complete grid, most passages): none.

## Per obstacle

The cells of an obstacle pooled. Unsafe runs: a contact or a clearance below 0.02 m, every run counted. The clearance columns are of the runs that reached (median, 5th percentile, minimum): a diagnostic conditioned on success, not the safety verdict.

| Obstacle | Mode | Margin (m) | Reached | Runs with contact | Unsafe runs | Unsafe and reached | Safe and reached | Clearance median (mm) | 5th percentile | Min |
|---|---|---|---|---|---|---|---|---|---|---|
| beam | ideal | 0.05 | 145/160 | 0 | 0 | 0 | 145 | 46.8 | 39.2 | 38.5 |
| beam | ideal | 0.04 | 146/160 | 0 | 2 | 2 | 144 | 32.1 | 28.6 | 3.7 |
| beam | ideal | 0.03 | 140/160 | 0 | 42 | 42 | 98 | 25.2 | 18.8 | 9.7 |
| beam | ideal | 0.02 | 152/160 | 5 | 111 | 110 | 42 | 10.1 | 8.0 | -12.0 |
| beam | realistic | 0.05 | 154/160 | 0 | 0 | 0 | 154 | 41.8 | 31.4 | 28.3 |
| beam | realistic | 0.04 | 151/160 | 0 | 7 | 7 | 144 | 29.2 | 20.0 | 15.3 |
| beam | realistic | 0.03 | 145/160 | 0 | 91 | 91 | 54 | 16.2 | 9.0 | 5.6 |
| beam | realistic | 0.02 | 145/160 | 10 | 109 | 106 | 39 | 8.4 | -0.2 | -1.3 |
| gap | ideal | 0.05 | 72/160 | 2 | 2 | 0 | 72 | 46.5 | 44.8 | 43.0 |
| gap | ideal | 0.04 | 75/160 | 1 | 2 | 0 | 75 | 36.4 | 34.8 | 33.4 |
| gap | ideal | 0.03 | 87/160 | 0 | 0 | 0 | 87 | 26.1 | 24.8 | 24.0 |
| gap | ideal | 0.02 | 87/160 | 0 | 141 | 87 | 0 | 16.0 | 14.5 | 13.2 |
| gap | realistic | 0.05 | 86/160 | 1 | 7 | 0 | 86 | 43.6 | 40.8 | 29.5 |
| gap | realistic | 0.04 | 106/160 | 1 | 4 | 1 | 105 | 33.8 | 30.8 | -0.2 |
| gap | realistic | 0.03 | 106/160 | 3 | 22 | 2 | 104 | 23.9 | 20.7 | 11.5 |
| gap | realistic | 0.02 | 97/160 | 1 | 147 | 96 | 1 | 14.0 | 10.4 | 2.7 |

## Adjacent margins on the same seeds

Exploratory. Seeds that reached at the smaller margin alone and at the larger alone; p: one-sided exact sign test, the smaller margin passing more; three steps a cell, no correction.

| Cell | Mode | Larger (m) | Smaller (m) | Seeds | Only at the smaller | Only at the larger | p |
|---|---|---|---|---|---|---|---|
| e3_beam_straight_1 | ideal | 0.05 | 0.04 | 20 | 0 | 0 | 1 |
| e3_beam_straight_1 | ideal | 0.04 | 0.03 | 20 | 0 | 0 | 1 |
| e3_beam_straight_1 | ideal | 0.03 | 0.02 | 20 | 0 | 0 | 1 |
| e3_beam_straight_1 | realistic | 0.05 | 0.04 | 20 | 0 | 0 | 1 |
| e3_beam_straight_1 | realistic | 0.04 | 0.03 | 20 | 0 | 0 | 1 |
| e3_beam_straight_1 | realistic | 0.03 | 0.02 | 20 | 0 | 0 | 1 |
| e3_beam_straight_2 | ideal | 0.05 | 0.04 | 20 | 2 | 0 | 0.25 |
| e3_beam_straight_2 | ideal | 0.04 | 0.03 | 20 | 0 | 0 | 1 |
| e3_beam_straight_2 | ideal | 0.03 | 0.02 | 20 | 0 | 0 | 1 |
| e3_beam_straight_2 | realistic | 0.05 | 0.04 | 20 | 0 | 0 | 1 |
| e3_beam_straight_2 | realistic | 0.04 | 0.03 | 20 | 0 | 1 | 1 |
| e3_beam_straight_2 | realistic | 0.03 | 0.02 | 20 | 1 | 0 | 0.5 |
| e3_beam_straight_3 | ideal | 0.05 | 0.04 | 20 | 1 | 2 | 0.875 |
| e3_beam_straight_3 | ideal | 0.04 | 0.03 | 20 | 0 | 2 | 1 |
| e3_beam_straight_3 | ideal | 0.03 | 0.02 | 20 | 4 | 0 | 0.0625 |
| e3_beam_straight_3 | realistic | 0.05 | 0.04 | 20 | 2 | 0 | 0.25 |
| e3_beam_straight_3 | realistic | 0.04 | 0.03 | 20 | 0 | 4 | 1 |
| e3_beam_straight_3 | realistic | 0.03 | 0.02 | 20 | 4 | 3 | 0.5 |
| e3_beam_straight_4 | ideal | 0.05 | 0.04 | 20 | 7 | 2 | 0.0898 |
| e3_beam_straight_4 | ideal | 0.04 | 0.03 | 20 | 2 | 5 | 0.938 |
| e3_beam_straight_4 | ideal | 0.03 | 0.02 | 20 | 5 | 0 | 0.0312 * |
| e3_beam_straight_4 | realistic | 0.05 | 0.04 | 20 | 0 | 1 | 1 |
| e3_beam_straight_4 | realistic | 0.04 | 0.03 | 20 | 1 | 2 | 0.875 |
| e3_beam_straight_4 | realistic | 0.03 | 0.02 | 20 | 3 | 2 | 0.5 |
| e3_beam_turn_1 | ideal | 0.05 | 0.04 | 20 | 0 | 0 | 1 |
| e3_beam_turn_1 | ideal | 0.04 | 0.03 | 20 | 0 | 0 | 1 |
| e3_beam_turn_1 | ideal | 0.03 | 0.02 | 20 | 0 | 1 | 1 |
| e3_beam_turn_1 | realistic | 0.05 | 0.04 | 20 | 0 | 1 | 1 |
| e3_beam_turn_1 | realistic | 0.04 | 0.03 | 20 | 1 | 0 | 0.5 |
| e3_beam_turn_1 | realistic | 0.03 | 0.02 | 20 | 0 | 2 | 1 |
| e3_beam_turn_2 | ideal | 0.05 | 0.04 | 20 | 1 | 2 | 0.875 |
| e3_beam_turn_2 | ideal | 0.04 | 0.03 | 20 | 2 | 3 | 0.812 |
| e3_beam_turn_2 | ideal | 0.03 | 0.02 | 20 | 3 | 1 | 0.312 |
| e3_beam_turn_2 | realistic | 0.05 | 0.04 | 20 | 0 | 1 | 1 |
| e3_beam_turn_2 | realistic | 0.04 | 0.03 | 20 | 1 | 1 | 0.75 |
| e3_beam_turn_2 | realistic | 0.03 | 0.02 | 20 | 1 | 0 | 0.5 |
| e3_beam_turn_3 | ideal | 0.05 | 0.04 | 20 | 2 | 1 | 0.5 |
| e3_beam_turn_3 | ideal | 0.04 | 0.03 | 20 | 1 | 2 | 0.875 |
| e3_beam_turn_3 | ideal | 0.03 | 0.02 | 20 | 2 | 2 | 0.688 |
| e3_beam_turn_3 | realistic | 0.05 | 0.04 | 20 | 1 | 3 | 0.938 |
| e3_beam_turn_3 | realistic | 0.04 | 0.03 | 20 | 3 | 2 | 0.5 |
| e3_beam_turn_3 | realistic | 0.03 | 0.02 | 20 | 2 | 1 | 0.5 |
| e3_beam_turn_4 | ideal | 0.05 | 0.04 | 20 | 1 | 6 | 0.992 |
| e3_beam_turn_4 | ideal | 0.04 | 0.03 | 20 | 5 | 4 | 0.5 |
| e3_beam_turn_4 | ideal | 0.03 | 0.02 | 20 | 5 | 3 | 0.363 |
| e3_beam_turn_4 | realistic | 0.05 | 0.04 | 20 | 2 | 2 | 0.688 |
| e3_beam_turn_4 | realistic | 0.04 | 0.03 | 20 | 2 | 4 | 0.891 |
| e3_beam_turn_4 | realistic | 0.03 | 0.02 | 20 | 3 | 6 | 0.91 |
| e3_gap_straight_1 | ideal | 0.05 | 0.04 | 20 | 2 | 3 | 0.812 |
| e3_gap_straight_1 | ideal | 0.04 | 0.03 | 20 | 3 | 4 | 0.773 |
| e3_gap_straight_1 | ideal | 0.03 | 0.02 | 20 | 4 | 4 | 0.637 |
| e3_gap_straight_1 | realistic | 0.05 | 0.04 | 20 | 3 | 0 | 0.125 |
| e3_gap_straight_1 | realistic | 0.04 | 0.03 | 20 | 0 | 1 | 1 |
| e3_gap_straight_1 | realistic | 0.03 | 0.02 | 20 | 1 | 1 | 0.75 |
| e3_gap_straight_2 | ideal | 0.05 | 0.04 | 20 | 4 | 2 | 0.344 |
| e3_gap_straight_2 | ideal | 0.04 | 0.03 | 20 | 6 | 3 | 0.254 |
| e3_gap_straight_2 | ideal | 0.03 | 0.02 | 20 | 2 | 7 | 0.98 |
| e3_gap_straight_2 | realistic | 0.05 | 0.04 | 20 | 2 | 2 | 0.688 |
| e3_gap_straight_2 | realistic | 0.04 | 0.03 | 20 | 3 | 3 | 0.656 |
| e3_gap_straight_2 | realistic | 0.03 | 0.02 | 20 | 2 | 3 | 0.812 |
| e3_gap_straight_3 | ideal | 0.05 | 0.04 | 20 | 2 | 5 | 0.938 |
| e3_gap_straight_3 | ideal | 0.04 | 0.03 | 20 | 7 | 1 | 0.0352 * |
| e3_gap_straight_3 | ideal | 0.03 | 0.02 | 20 | 5 | 5 | 0.623 |
| e3_gap_straight_3 | realistic | 0.05 | 0.04 | 20 | 5 | 3 | 0.363 |
| e3_gap_straight_3 | realistic | 0.04 | 0.03 | 20 | 4 | 2 | 0.344 |
| e3_gap_straight_3 | realistic | 0.03 | 0.02 | 20 | 3 | 7 | 0.945 |
| e3_gap_straight_4 | ideal | 0.05 | 0.04 | 20 | 1 | 1 | 0.75 |
| e3_gap_straight_4 | ideal | 0.04 | 0.03 | 20 | 3 | 1 | 0.312 |
| e3_gap_straight_4 | ideal | 0.03 | 0.02 | 20 | 3 | 2 | 0.5 |
| e3_gap_straight_4 | realistic | 0.05 | 0.04 | 20 | 2 | 2 | 0.688 |
| e3_gap_straight_4 | realistic | 0.04 | 0.03 | 20 | 7 | 1 | 0.0352 * |
| e3_gap_straight_4 | realistic | 0.03 | 0.02 | 20 | 4 | 5 | 0.746 |
| e3_gap_turn_1 | ideal | 0.05 | 0.04 | 20 | 4 | 4 | 0.637 |
| e3_gap_turn_1 | ideal | 0.04 | 0.03 | 20 | 3 | 2 | 0.5 |
| e3_gap_turn_1 | ideal | 0.03 | 0.02 | 20 | 3 | 11 | 0.994 |
| e3_gap_turn_1 | realistic | 0.05 | 0.04 | 20 | 3 | 3 | 0.656 |
| e3_gap_turn_1 | realistic | 0.04 | 0.03 | 20 | 3 | 6 | 0.91 |
| e3_gap_turn_1 | realistic | 0.03 | 0.02 | 20 | 4 | 2 | 0.344 |
| e3_gap_turn_2 | ideal | 0.05 | 0.04 | 20 | 8 | 5 | 0.291 |
| e3_gap_turn_2 | ideal | 0.04 | 0.03 | 20 | 6 | 7 | 0.709 |
| e3_gap_turn_2 | ideal | 0.03 | 0.02 | 20 | 6 | 1 | 0.0625 |
| e3_gap_turn_2 | realistic | 0.05 | 0.04 | 20 | 10 | 0 | 0.000977 * |
| e3_gap_turn_2 | realistic | 0.04 | 0.03 | 20 | 1 | 7 | 0.996 |
| e3_gap_turn_2 | realistic | 0.03 | 0.02 | 20 | 4 | 4 | 0.637 |
| e3_gap_turn_3 | ideal | 0.05 | 0.04 | 20 | 5 | 3 | 0.363 |
| e3_gap_turn_3 | ideal | 0.04 | 0.03 | 20 | 7 | 5 | 0.387 |
| e3_gap_turn_3 | ideal | 0.03 | 0.02 | 20 | 5 | 0 | 0.0312 * |
| e3_gap_turn_3 | realistic | 0.05 | 0.04 | 20 | 8 | 2 | 0.0547 |
| e3_gap_turn_3 | realistic | 0.04 | 0.03 | 20 | 5 | 4 | 0.5 |
| e3_gap_turn_3 | realistic | 0.03 | 0.02 | 20 | 3 | 6 | 0.91 |
| e3_gap_turn_4 | ideal | 0.05 | 0.04 | 20 | 2 | 2 | 0.688 |
| e3_gap_turn_4 | ideal | 0.04 | 0.03 | 20 | 1 | 1 | 0.75 |
| e3_gap_turn_4 | ideal | 0.03 | 0.02 | 20 | 4 | 2 | 0.344 |
| e3_gap_turn_4 | realistic | 0.05 | 0.04 | 20 | 1 | 2 | 0.875 |
| e3_gap_turn_4 | realistic | 0.04 | 0.03 | 20 | 2 | 1 | 0.5 |
| e3_gap_turn_4 | realistic | 0.03 | 0.02 | 20 | 1 | 3 | 0.938 |

## Realistic against ideal

The ideal plant executes the commands as sent, and B5 plans there with the unicycle: the two modes differ in the plant and in B5's prediction model, so a difference is of the two packages, not of tracking alone. p: one-sided Fisher per cell and margin, the realistic plant passing fewer, uncorrected; a p above 0.05 is no detected deficit, not equivalence.

| Cell | Margin (m) | Realistic | Ideal | p |
|---|---|---|---|---|
| e3_beam_straight_1 | 0.05 | 20/20 | 20/20 | 1 |
| e3_beam_straight_1 | 0.04 | 20/20 | 20/20 | 1 |
| e3_beam_straight_1 | 0.03 | 20/20 | 20/20 | 1 |
| e3_beam_straight_1 | 0.02 | 20/20 | 20/20 | 1 |
| e3_beam_straight_2 | 0.05 | 20/20 | 18/20 | 1 |
| e3_beam_straight_2 | 0.04 | 20/20 | 20/20 | 1 |
| e3_beam_straight_2 | 0.03 | 19/20 | 20/20 | 0.5 |
| e3_beam_straight_2 | 0.02 | 20/20 | 20/20 | 1 |
| e3_beam_straight_3 | 0.05 | 18/20 | 19/20 | 0.5 |
| e3_beam_straight_3 | 0.04 | 20/20 | 18/20 | 1 |
| e3_beam_straight_3 | 0.03 | 16/20 | 16/20 | 0.653 |
| e3_beam_straight_3 | 0.02 | 17/20 | 20/20 | 0.115 |
| e3_beam_straight_4 | 0.05 | 19/20 | 13/20 | 0.998 |
| e3_beam_straight_4 | 0.04 | 18/20 | 18/20 | 0.698 |
| e3_beam_straight_4 | 0.03 | 17/20 | 15/20 | 0.882 |
| e3_beam_straight_4 | 0.02 | 18/20 | 20/20 | 0.244 |
| e3_beam_turn_1 | 0.05 | 20/20 | 20/20 | 1 |
| e3_beam_turn_1 | 0.04 | 19/20 | 20/20 | 0.5 |
| e3_beam_turn_1 | 0.03 | 20/20 | 20/20 | 1 |
| e3_beam_turn_1 | 0.02 | 18/20 | 19/20 | 0.5 |
| e3_beam_turn_2 | 0.05 | 20/20 | 19/20 | 1 |
| e3_beam_turn_2 | 0.04 | 19/20 | 18/20 | 0.885 |
| e3_beam_turn_2 | 0.03 | 19/20 | 17/20 | 0.947 |
| e3_beam_turn_2 | 0.02 | 20/20 | 19/20 | 1 |
| e3_beam_turn_3 | 0.05 | 19/20 | 18/20 | 0.885 |
| e3_beam_turn_3 | 0.04 | 17/20 | 19/20 | 0.302 |
| e3_beam_turn_3 | 0.03 | 18/20 | 18/20 | 0.698 |
| e3_beam_turn_3 | 0.02 | 19/20 | 18/20 | 0.885 |
| e3_beam_turn_4 | 0.05 | 18/20 | 18/20 | 0.698 |
| e3_beam_turn_4 | 0.04 | 18/20 | 13/20 | 0.99 |
| e3_beam_turn_4 | 0.03 | 16/20 | 14/20 | 0.863 |
| e3_beam_turn_4 | 0.02 | 13/20 | 16/20 | 0.24 |
| e3_gap_straight_1 | 0.05 | 17/20 | 18/20 | 0.5 |
| e3_gap_straight_1 | 0.04 | 20/20 | 17/20 | 1 |
| e3_gap_straight_1 | 0.03 | 19/20 | 16/20 | 0.976 |
| e3_gap_straight_1 | 0.02 | 19/20 | 16/20 | 0.976 |
| e3_gap_straight_2 | 0.05 | 17/20 | 12/20 | 0.985 |
| e3_gap_straight_2 | 0.04 | 17/20 | 14/20 | 0.936 |
| e3_gap_straight_2 | 0.03 | 17/20 | 17/20 | 0.669 |
| e3_gap_straight_2 | 0.02 | 16/20 | 12/20 | 0.959 |
| e3_gap_straight_3 | 0.05 | 13/20 | 7/20 | 0.987 |
| e3_gap_straight_3 | 0.04 | 15/20 | 4/20 | 1 |
| e3_gap_straight_3 | 0.03 | 17/20 | 10/20 | 0.997 |
| e3_gap_straight_3 | 0.02 | 13/20 | 10/20 | 0.9 |
| e3_gap_straight_4 | 0.05 | 2/20 | 1/20 | 0.885 |
| e3_gap_straight_4 | 0.04 | 2/20 | 1/20 | 0.885 |
| e3_gap_straight_4 | 0.03 | 8/20 | 3/20 | 0.985 |
| e3_gap_straight_4 | 0.02 | 7/20 | 4/20 | 0.922 |
| e3_gap_turn_1 | 0.05 | 17/20 | 15/20 | 0.882 |
| e3_gap_turn_1 | 0.04 | 17/20 | 15/20 | 0.882 |
| e3_gap_turn_1 | 0.03 | 14/20 | 16/20 | 0.358 |
| e3_gap_turn_1 | 0.02 | 16/20 | 8/20 | 0.998 |
| e3_gap_turn_2 | 0.05 | 9/20 | 11/20 | 0.376 |
| e3_gap_turn_2 | 0.04 | 19/20 | 14/20 | 0.996 |
| e3_gap_turn_2 | 0.03 | 13/20 | 13/20 | 0.629 |
| e3_gap_turn_2 | 0.02 | 13/20 | 18/20 | 0.0637 |
| e3_gap_turn_3 | 0.05 | 8/20 | 5/20 | 0.912 |
| e3_gap_turn_3 | 0.04 | 14/20 | 7/20 | 0.995 |
| e3_gap_turn_3 | 0.03 | 15/20 | 9/20 | 0.989 |
| e3_gap_turn_3 | 0.02 | 12/20 | 14/20 | 0.371 |
| e3_gap_turn_4 | 0.05 | 3/20 | 3/20 | 0.669 |
| e3_gap_turn_4 | 0.04 | 2/20 | 3/20 | 0.5 |
| e3_gap_turn_4 | 0.03 | 3/20 | 3/20 | 0.669 |
| e3_gap_turn_4 | 0.02 | 1/20 | 5/20 | 0.0909 |

| Cell | Smallest safe margin, realistic (m) | Ideal (m) |
|---|---|---|
| e3_beam_straight_1 | 0.05 | 0.05 |
| e3_beam_straight_2 | 0.04 | 0.04 |
| e3_beam_straight_3 | 0.04 | 0.04 |
| e3_beam_straight_4 | 0.04 | 0.04 |
| e3_beam_turn_1 | 0.05 | 0.04 |
| e3_beam_turn_2 | 0.05 | 0.04 |
| e3_beam_turn_3 | 0.05 | 0.04 |
| e3_beam_turn_4 | 0.05 | 0.05 |
| e3_gap_straight_1 | 0.04 | 0.03 |
| e3_gap_straight_2 | 0.04 | 0.03 |
| e3_gap_straight_3 | 0.04 | 0.03 |
| e3_gap_straight_4 | - | 0.03 |
| e3_gap_turn_1 | 0.04 | 0.03 |
| e3_gap_turn_2 | 0.04 | 0.03 |
| e3_gap_turn_3 | 0.04 | 0.03 |
| e3_gap_turn_4 | - | 0.03 |

## Straight against turn

A description, not a criterion: the pair differs in geometry as well as in motion.

| Geometry | Step | Mode | Smallest safe margin, straight (m) | Turn (m) | Turn minus straight (m) |
|---|---|---|---|---|---|
| beam | 1 | ideal | 0.05 | 0.04 | -0.01 |
| beam | 1 | realistic | 0.05 | 0.05 | 0.00 |
| beam | 2 | ideal | 0.04 | 0.04 | 0.00 |
| beam | 2 | realistic | 0.04 | 0.05 | 0.01 |
| beam | 3 | ideal | 0.04 | 0.04 | 0.00 |
| beam | 3 | realistic | 0.04 | 0.05 | 0.01 |
| beam | 4 | ideal | 0.04 | 0.05 | 0.01 |
| beam | 4 | realistic | 0.04 | 0.05 | 0.01 |
| gap | 1 | ideal | 0.03 | 0.03 | 0.00 |
| gap | 1 | realistic | 0.04 | 0.04 | 0.00 |
| gap | 2 | ideal | 0.03 | 0.03 | 0.00 |
| gap | 2 | realistic | 0.04 | 0.04 | 0.00 |
| gap | 3 | ideal | 0.03 | 0.03 | 0.00 |
| gap | 3 | realistic | 0.04 | 0.04 | 0.00 |
| gap | 4 | ideal | 0.03 | 0.03 | 0.00 |
| gap | 4 | realistic | - | - | - |

## Informative cells

A cell where every margin passes every run, every margin passes none, or reached is not ordered in the margin is uninformative and takes no part in a threshold. The rule is an eligibility screen fixed before the sweep: a flat pass rate counts as informative, so the flag is not evidence of a threshold.

| Cell | Mode | Informative |
|---|---|---|
| e3_beam_straight_1 | ideal | uninformative |
| e3_beam_straight_1 | realistic | uninformative |
| e3_beam_straight_2 | ideal | yes |
| e3_beam_straight_2 | realistic | yes |
| e3_beam_straight_3 | ideal | yes |
| e3_beam_straight_3 | realistic | uninformative |
| e3_beam_straight_4 | ideal | uninformative |
| e3_beam_straight_4 | realistic | yes |
| e3_beam_turn_1 | ideal | yes |
| e3_beam_turn_1 | realistic | yes |
| e3_beam_turn_2 | ideal | yes |
| e3_beam_turn_2 | realistic | yes |
| e3_beam_turn_3 | ideal | yes |
| e3_beam_turn_3 | realistic | yes |
| e3_beam_turn_4 | ideal | uninformative |
| e3_beam_turn_4 | realistic | uninformative |
| e3_gap_straight_1 | ideal | yes |
| e3_gap_straight_1 | realistic | yes |
| e3_gap_straight_2 | ideal | uninformative |
| e3_gap_straight_2 | realistic | yes |
| e3_gap_straight_3 | ideal | uninformative |
| e3_gap_straight_3 | realistic | uninformative |
| e3_gap_straight_4 | ideal | yes |
| e3_gap_straight_4 | realistic | yes |
| e3_gap_turn_1 | ideal | uninformative |
| e3_gap_turn_1 | realistic | uninformative |
| e3_gap_turn_2 | ideal | yes |
| e3_gap_turn_2 | realistic | uninformative |
| e3_gap_turn_3 | ideal | yes |
| e3_gap_turn_3 | realistic | uninformative |
| e3_gap_turn_4 | ideal | yes |
| e3_gap_turn_4 | realistic | yes |
