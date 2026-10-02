# If Rx/Tx pulses start to miss or timeout

This is a sympton on TX and RX no being synchronized.
The solution is to adjust START_OFFSET_S in params.json until the problem is not observed

For 1s, 0.15s is recommended.

For 25s, 2.0s is recommended.