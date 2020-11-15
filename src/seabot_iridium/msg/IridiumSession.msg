
# 0 MO message, if any, transferred successfully.
# 1 MO message, if any, transferred successfully, but the MT message in the queue was too big to be transferred.
# 2 MO message, if any, transferred successfully, but the requested Location Update was not accepted.
# 3..4 Reserved, but indicate MO session success if used.
# 5..8 Reserved, but indicate MO session failure if used.
# 10 GSS reported that the call did not complete in the allowed time.
# 11 MO message queue at the GSS is full.
# 12 MO message has too many segments.
# 13 GSS reported that the session did not complete.
# 14 Invalid segment size.
# 15 Access is denied.

# ISU-reported values:
# 16 ISU has been locked and may not make SBD calls (see +CULK command).
# 17 Gateway not responding (local session timeout).
# 18 Connection lost (RF drop).
# 19 Link failure (A protocol error caused termination of the call).
# 20..31 Reserved, but indicate failure if used.
# 32 No network service, unable to initiate call.
# 33 Antenna fault, unable to initiate call.
# 34 Radio is disabled, unable to initiate call (see *Rn command).
# 35 ISU is busy, unable to initiate call.
# 36 Try later, must wait 3 minutes since last registration.
# 37 SBD service is temporarily disabled.
# 38 Try later, traffic management period (see +SBDLOE command)
# 39..63 Reserved, but indicate failure if used.
# 64 Band violation (attempt to transmit outside permitted frequency band).
# 65 PLL lock failure; hardware error during attempted transmit.
int8 mo

# range from 0 to 65535
int8 momsn

# 0 No SBD message to receive from the GSS.
# 1 SBD message successfully received from the GSS.
# 2 An error occurred while attempting to perform a mailbox check or receive a message from the GSS.
int8 mt

# range from 0 to 65535
int8 mtmsn

int8 waiting
