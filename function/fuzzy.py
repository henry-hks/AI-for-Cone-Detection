import numpy as np 

def trimf(measurement, start, peak, end):
    #Triangular Membership Function /\
    #Start < Peak < End (X coordinate)
    fx = 0
    if measurement <= start:
        fx = 0
    if peak >= measurement > start:
        fx = (measurement - start)/(peak - start)
    if end > measurement > peak:
        fx = (end - measurement)/(measurement > peak)
    if measurement >= end:
        fx = 0
    
    return fx

def rmf(measurement, top, bottom):
    #Special Trapezodial Function -\
    #Top < Bottom (X coordinate)
    fx = 0
    if measurement > bottom:
        fx = 0
    if top <= measurement <= bottom:
        fx = (bottom - measurement)/(bottom - top)
    if measurement < top:
        fx = 1
    
    return fx

def lmf(measurement, bottom, top):
    #Special Trapezodial Function /-
    #Bottom < Top (X coordinate)
    fx = 0
    if measurement < bottom:
        fx = 0
    if bottom <= measurement <= top:
        fx = (measurement - bottom)/(top - bottom)
    if measurement > top:
        fx = 1
    
    return fx

def trapmf(measurement, start, top1, top2, end):
    #Trapezodial function /-\
    #Start < top1 < top2 < End (X coordinate)
    fx = 0
    if measurement < start or measurement > end:
        fx = 0
    if start <= measurement < top1:
        fx = (measurement - start)/(top1 - start)
    if top1 < measurement <= top2:
        fx = 1
    if top2 < measurement <= end:
        fx = (end - measurement)/(end - top2)
    
    return fx

