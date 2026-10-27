"""预分配数值日志；运动期间不保留逐行Python对象，结束后流式导出。"""
from array import array
import math


class NumericBuffer:
    def __init__(self, header, capacity=20000):
        self.header=list(header)
        self.width=len(header)
        self.capacity=capacity
        self.count=0
        self.data=array('d', [0.])*(capacity*self.width)

    def writerow(self, row):
        if self.count>=self.capacity or len(row)!=self.width:
            raise RuntimeError('数值日志缓存容量/列数不符')
        start=self.count*self.width
        for j,value in enumerate(row):
            self.data[start+j]=float(value) if value!='' else math.nan
        self.count+=1

    def column(self,j):
        return [self.data[i*self.width+j] for i in range(self.count)]

    def export_rows(self):
        yield self.header
        for i in range(self.count):
            yield ['' if math.isnan(x) else x
                   for x in self.data[i*self.width:(i+1)*self.width]]
