# This message holds battery status
#

# Possible levels of operations
byte OK=0
byte WARNING=1
byte CRITICAL=2

byte level                      # level of operation enumerated above
# uint64 max_capacity             # [mAh]
# float32 voltage                 # [V]
float32 current                 # [A]

float32 remaining_percentage    # [%]
float32 remaining_capacity       # [mAh]
