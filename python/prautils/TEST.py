#!/usr/bin/env python

"""
Test module
"""
import os

class Student(object):
    """
    A class defining students
    """
    # They all go to SSPS
    school = 'SSPS'
    school_status = 'TOTAL SHIT'

    def __init__(self, name, rank):
        self.__name = name
        self.rank = rank

    @classmethod
    def upgraderank(cls, steps):
        self.rank = self.rank - steps
        print 'rank is now ', self.rank

    def printname(self):
        print 'Student name is: ', self.__name

    @classmethod
    def printschool(cls):
        print 'Student goes to: ', Student.school
        print 'This school is: ', Student.school_status
