# Rolling timing statistics over the last status window (milliseconds).
string name
uint64 samples
float64 mean_ms
float64 p95_ms
float64 p99_ms
float64 max_ms
float64 budget_ms        # deadline used for the overrun count
uint64 overruns_window   # samples above budget_ms in this window
uint64 overruns_total    # samples above budget_ms since start
