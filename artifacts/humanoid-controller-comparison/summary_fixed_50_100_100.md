| Scenario | Controller | Success / ticks | Task-valid ticks | Progress (m) | Avg. task deviation | Avg. invariant deviation | Avg. joint-limit clearance (rad) | Avg. waist-corridor clearance (m) | Mean / P95 solve time (ms) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| absolute | QP | yes / 30 | 23 / 50 | 0.107 | 5.76\times 10^{-5} | 3.70\times 10^{-3} | 0.029 | n/a | 25.8 / 25.5 |
| absolute | HQP | yes / 30 | 10 / 50 | 0.106 | 5.00\times 10^{-5} | 5.58\times 10^{-4} | 0.000 | n/a | 26.6 / 31.6 |
| absolute | SNS | yes / 30 | 10 / 50 | 0.106 | 5.02\times 10^{-5} | 5.52\times 10^{-4} | 0.000 | n/a | 24.6 / 26.9 |
| absolute | Predictive IK | yes / 30 | 43 / 50 | 0.107 | 1.64\times 10^{-4} | 9.60\times 10^{-3} | 0.000 | n/a | 39.1 / 55.0 |
| absolute | Recovery Optimization (TR) | yes / 30 | 42 / 50 | 0.106 | 7.02\times 10^{-6} | 1.91\times 10^{-8} | 0.000 | n/a | 38.3 / 65.6 |
| absolute | Recovery Optimization (TR+Safety Bias) | yes / 30 | 40 / 50 | 0.100 | 3.41\times 10^{-5} | 5.24\times 10^{-6} | 0.037 | n/a | 68.8 / 155.1 |
| decoupled-upright | QP | no / n/a | 4 / 100 | 0.242 | 7.98\times 10^{-6} | 2.92\times 10^{-8} | 0.000 | 0.093 | 24.6 / 31.1 |
| decoupled-upright | HQP | no / n/a | 98 / 100 | 0.250 | 6.83\times 10^{-7} | 5.31\times 10^{-4} | 0.000 | 0.094 | 28.7 / 38.7 |
| decoupled-upright | SNS | no / n/a | 98 / 100 | 0.250 | 6.45\times 10^{-7} | 7.04\times 10^{-4} | 0.000 | 0.094 | 28.8 / 33.4 |
| decoupled-upright | Predictive IK | no / n/a | 7 / 100 | 0.249 | 6.52\times 10^{-6} | 1.68\times 10^{-4} | 0.000 | 0.094 | 47.4 / 54.9 |
| decoupled-upright | Recovery Optimization (TR) | no / n/a | 100 / 100 | 0.250 | 5.92\times 10^{-9} | 3.30\times 10^{-8} | 0.000 | 0.111 | 38.1 / 55.5 |
| decoupled-upright | Recovery Optimization (TR+Safety Bias) | no / n/a | 100 / 100 | 0.250 | 5.74\times 10^{-9} | 1.14\times 10^{-8} | 0.037 | 0.109 | 88.7 / 147.4 |
| waist-distance | QP | yes / 30 | 0 / 100 | 0.250 | 1.66\times 10^{-6} | 6.38\times 10^{-9} | 0.000 | 0.183 | 23.1 / 26.7 |
| waist-distance | HQP | yes / 30 | 0 / 100 | 0.250 | 1.90\times 10^{-6} | 4.20\times 10^{-8} | 0.000 | 0.183 | 25.0 / 27.5 |
| waist-distance | SNS | yes / 30 | 0 / 100 | 0.250 | 1.90\times 10^{-6} | 4.20\times 10^{-8} | 0.000 | 0.183 | 23.8 / 26.6 |
| waist-distance | Predictive IK | yes / 30 | 0 / 100 | 0.250 | 1.66\times 10^{-6} | 6.73\times 10^{-9} | 0.000 | 0.183 | 51.0 / 54.7 |
| waist-distance | Recovery Optimization (TR) | yes / 30 | 100 / 100 | 0.250 | 4.78\times 10^{-10} | 8.88\times 10^{-9} | 0.000 | 0.183 | 52.3 / 55.8 |
| waist-distance | Recovery Optimization (TR+Safety Bias) | yes / 30 | 100 / 100 | 0.250 | 2.19\times 10^{-10} | 8.88\times 10^{-9} | 0.036 | 0.178 | 50.8 / 55.4 |
