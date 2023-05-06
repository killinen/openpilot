#!/usr/bin/env python3
from raven import Client

client = Client('https://fb79b6ca14ba40ec9f4ce876618a544f@o1107536.ingest.sentry.io/6135176')

try:
	1 / 0
except ZeroDivisionError:
	client.captureException()
