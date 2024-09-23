#!/usr/bin/env python

import random as r

# Script specifically for generating random addition equations for
# distraction during user testing

def main(numEqs=30):

    for i in range(numEqs):
        n1 = int(str(r.randint(0, 9)) + str(r.randint(0, 9)))
        n2 = int(str(r.randint(0, 9)) + str(r.randint(0, 9)))
        ans = n1 + n2

        print(str(i + 1) + ') ' + str(n1) + ' + ' + str(n2) + ' = ' + str(ans))

if __name__=='__main__':
    main()
