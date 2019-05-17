import math
import numpy as np

def commaSeparated(sequence, blockGenerator):
    count = 0
    suffix = ","

    for item in sequence :
        if count == len(sequence)-1 : suffix = ""
        yield blockGenerator(item) + suffix
        count = count + 1


def numericArrayToText(numeric, formatter):
    text = np.empty( shape=np.shape(numeric), dtype='U16')
    for index in np.ndindex( np.shape(numeric) ) :
        text[index] = formatter.float2str( numeric[index] )
    return text


class FloatsFormatter :
    def __init__(self, round_digits=6, pi_round_digits=5, pi_string="pi"):
        self.round_decimals = round_digits
        self.pi_round_decimals = pi_round_digits
        self.pi_string = pi_string

        self.roundedPI     = round(math.pi,   self.pi_round_decimals)
        self.roundedHalfPI = round(math.pi/2, self.pi_round_decimals)

        self.formatStr = '0:' + str(self.round_decimals)

    def float2str(self, num, angle=False ) :
        if angle :
            value = round(num, self.pi_round_decimals)
            sign  = "-" if value<0 else ""
            if abs(value) == self.roundedPI :
                return sign + self.pi_string
            if abs(value) == self.roundedHalfPI :
                return sign + self.pi_string + "/2.0"

        num = round(num, self.round_decimals)
        num += 0  # this trick avoids the annoying '-0.0' (minus zero)

        return( ( "{" + self.formatStr + "}" ).format( num ) )

