import math
from fractions import Fraction

# Utils


def gcd(a, b):
    return math.gcd(a, b)


def lcm(a, b):
    return abs(a * b) // gcd(a, b)


def round_time(t, decimals=6):
    return round(t, decimals)


def gcm_of_floats(float1, float2):
    frac1 = Fraction(float1).limit_denominator()
    frac2 = Fraction(float2).limit_denominator()

    gcd_numerators = gcd(frac1.numerator, frac2.numerator)
    lcm_denominators = lcm(frac1.denominator, frac2.denominator)

    gcm_fraction = Fraction(gcd_numerators, lcm_denominators)
    return float(gcm_fraction)
