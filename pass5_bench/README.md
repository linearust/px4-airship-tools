# Bench verification of 81f72e4920 (2026-09-17 evening, fmu-v6x)

The whole day's work re-measured at the pins, against the 2026-09-16 reference taken
straight after pass 9.

- firmware: airship/dev tip 81f72e4920, which is the day's five refactor passes AND a
  rebase onto 90 newer upstream commits
- board reconfigured to airframe **2510** (renumbered today from 2504); the bench
  parameter set survived the switch unchanged
- 13 stick vectors x 8 channels, median pulse width over the last second of each hold

**max |diff| 0.0 us, 0 of 13 vectors differ.**

So: the rear-cone rewrite, the dead-branch removal, the discountHeld inline, the
forwarder removal and 90 upstream commits together change nothing a servo or an ESC
can see.
