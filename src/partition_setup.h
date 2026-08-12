#pragma once

// Creates or safely enlarges the named Mac Plus data partition after explicit
// keyboard confirmation. Returns normally when no partition-table write was
// needed or the user skipped it; a successful change restarts the device.
void macplusStorageSetup();
