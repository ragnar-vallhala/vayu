# LED & Buzzer Patterns

The Vayu flight controller uses a combination of three LEDs (Blue, Green, Red) and a Buzzer to provide real-time status and diagnostic feedback.

## Normal Operational Patterns

| System State   | Visual / Audible Signal                | Meaning                                             |
| :------------- | :------------------------------------- | :-------------------------------------------------- |
| **INIT**       | 100ms Startup Beep + **Blue** Toggling | System is booting and running self-checks.          |
| **STANDBY**    | **Green** Toggling                     | All checks passed. Ready for pilot input.           |
| **PREARM**     | **Green** + **Blue** Toggling          | Pilot command received, checking safety conditions. |
| **ARMED**      | **Green** Toggling + **Red** Solid     | Motors live. Takeoff permitted.                     |
| **IN_AIR**     | **Green** + **Red** Toggling           | Flight mode active.                                 |
| **TERMINATED** | **Red** Solid + **Buzzer** Solid       | System shutdown or emergency stop.                  |

## Failsafe & Error Patterns

If a critical error is detected, the system enters **FAILSAFE** mode.

| Signal               | Meaning                             |
| :------------------- | :---------------------------------- |
| **Blinking Red**     | Master Failsafe indication.         |
| **Rhythmic Beeping** | Audible alert accompanying Red LED. |

### Diagnostic Codes (during Failsafe)

While the Red LED is blinking, the following solid LEDs identify the specific failure:

| Solid LED       | Failure Reason                                               |
| :-------------- | :----------------------------------------------------------- |
| **Solid BLUE**  | **System Clock Error**: Frequency mismatch (expected 84MHz). |
| **Solid GREEN** | **SD Card Error**: Handshake or initialization failed.       |
| **Both Solid**  | Multiple boot checks failed.                                 |
