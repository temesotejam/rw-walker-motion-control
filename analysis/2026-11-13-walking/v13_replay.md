# v13 recorded-trajectory candidate comparison

Recorded old commands/peaks force-update the controller and wheel guard. New physical response, full driver/current/hardware health and precise scheduling are not simulated. Each first candidate is independent.

| Log | Added publication delay | v12 | Turn only | Amount only | Both |
|---|---:|---:|---:|---:|---:|
| 141282685 | 0 ms | 3 | 3 | 3 | 3 |
| 141282685 | 10 ms | 3 | 3 | 3 | 3 |
| 141282685 | 25 ms | 0 | 0 | 0 | 0 |
| 137267573 | 0 ms | 5 | 5 | 5 | 5 |
| 137267573 | 10 ms | 5 | 5 | 5 | 5 |
| 137267573 | 25 ms | 4 | 3 | 3 | 3 |
| 267557373 | 0 ms | 1 | 1 | 1 | 1 |
| 267557373 | 10 ms | 1 | 1 | 1 | 1 |
| 267557373 | 25 ms | 1 | 1 | 1 | 1 |
| 27908524 | 0 ms | 2 | 2 | 2 | 2 |
| 27908524 | 10 ms | 2 | 2 | 2 | 2 |
| 27908524 | 25 ms | 1 | 1 | 1 | 1 |
| 64040782 | 0 ms | 1 | 1 | 1 | 1 |
| 64040782 | 10 ms | 1 | 1 | 1 | 1 |
| 64040782 | 25 ms | 1 | 1 | 1 | 1 |
| 84815132 | 0 ms | 5 | 5 | 5 | 5 |
| 84815132 | 10 ms | 5 | 5 | 5 | 5 |
| 84815132 | 25 ms | 4 | 4 | 4 | 4 |
| 57814129 | 0 ms | 0 | 0 | 0 | 0 |
| 57814129 | 10 ms | 0 | 0 | 0 | 0 |
| 57814129 | 25 ms | 0 | 0 | 0 | 0 |
| 199015154 | 0 ms | 10 | 9 | 8 | 8 |
| 199015154 | 10 ms | 8 | 7 | 7 | 7 |
| 199015154 | 25 ms | 6 | 5 | 5 | 5 |
| 611622951 | 0 ms | 3 | 3 | 3 | 3 |
| 611622951 | 10 ms | 0 | 1 | 0 | 1 |
| 611622951 | 25 ms | 0 | 1 | 0 | 1 |
| 43750941 | 0 ms | 0 | 0 | 0 | 0 |
| 43750941 | 10 ms | 0 | 0 | 0 | 0 |
| 43750941 | 25 ms | 0 | 0 | 0 | 0 |

## First candidates after final large negative peak

| Log | Delay | Variant | Time | Source | Raw ordinary | Added | Raw backend | Guarded | Width match |
|---|---:|---|---:|---:|---:|---:|---:|---:|---|
| 611622951 | 0 | v12 | 28.854 s | 2 | 56 | 0 | 56 | 56 | True |
| 611622951 | 0 | v13_turn_only | 28.753 s | 3 | 56 | 0 | 56 | 56 | True |
| 611622951 | 0 | v13_amount_only | 28.854 s | 2 | 56 | 8 | 64 | 64 | True |
| 611622951 | 0 | v13_turn_and_amount | 28.753 s | 3 | 56 | 10 | 66 | 66 | True |
| 611622951 | 10 | v12 | none | - | - | - | - | - | - |
| 611622951 | 10 | v13_turn_only | 28.764 s | 3 | 56 | 0 | 56 | 56 | True |
| 611622951 | 10 | v13_amount_only | none | - | - | - | - | - | - |
| 611622951 | 10 | v13_turn_and_amount | 28.764 s | 3 | 56 | 10 | 66 | 66 | True |
| 611622951 | 25 | v12 | none | - | - | - | - | - | - |
| 611622951 | 25 | v13_turn_only | 28.775 s | 3 | 56 | 0 | 56 | 56 | True |
| 611622951 | 25 | v13_amount_only | none | - | - | - | - | - | - |
| 611622951 | 25 | v13_turn_and_amount | 28.775 s | 3 | 56 | 10 | 66 | 66 | True |
| 141282685 | 0 | v12 | 27.921 s | 2 | 70 | 0 | 70 | 68 | True |
| 141282685 | 0 | v13_turn_only | 27.921 s | 2 | 70 | 0 | 70 | 68 | True |
| 141282685 | 0 | v13_amount_only | 27.921 s | 2 | 70 | 4 | 74 | 72 | True |
| 141282685 | 0 | v13_turn_and_amount | 27.921 s | 2 | 70 | 4 | 74 | 72 | True |
| 141282685 | 10 | v12 | 27.927 s | 2 | 70 | 0 | 70 | 68 | True |
| 141282685 | 10 | v13_turn_only | 27.927 s | 2 | 70 | 0 | 70 | 68 | True |
| 141282685 | 10 | v13_amount_only | 27.927 s | 2 | 70 | 4 | 74 | 72 | True |
| 141282685 | 10 | v13_turn_and_amount | 27.927 s | 2 | 70 | 4 | 74 | 72 | True |
| 141282685 | 25 | v12 | none | - | - | - | - | - | - |
| 141282685 | 25 | v13_turn_only | none | - | - | - | - | - | - |
| 141282685 | 25 | v13_amount_only | none | - | - | - | - | - | - |
| 141282685 | 25 | v13_turn_and_amount | none | - | - | - | - | - | - |

Source 3 is tentative first reversal, not confirmed Returning or contact. Width addition is an experimental heuristic, not identified required work. Counts on already assisted v12 trajectories are not an unassisted control or a proof of benefit.
