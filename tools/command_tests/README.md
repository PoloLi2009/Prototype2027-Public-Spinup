# Shared command checks

Run `python3 tools/command_tests/run.py` from the Spinup checkout. This uses locally cached FTCLib 2.1.1 and Pedro 3.0.2 with deterministic fake devices; it does not connect to a robot or download dependencies. `JAVA_HOME` and `GRADLE_USER_HOME` can override the local defaults.

The optional `--baseline` argument enables replay against a pre-change SequencedShotCommand.java. Beside that file, provide `commands-before/TransitShooterManualCommand.java` for the independent manual fixture. Production sources are never copied into the checkout; temporary compiled classes are removed automatically.

See `docs/COMMANDS.md` for APIs, ownership, lifecycle and validation limits.
