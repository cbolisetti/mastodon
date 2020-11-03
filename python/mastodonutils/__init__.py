"""General utilities for MASTODON"""
import sys
if not (sys.version_info[0]==2):
    ver = sys.version_info[0]
    raise Exception ('Failed to execute. The MASTODON Python module currently runs on Python2. You are currently using Python{}.'.format(ver))
