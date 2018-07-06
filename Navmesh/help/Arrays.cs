using System;
using System.Collections.Generic;
using System.Text;


public class Arrays
{


    public static void fill(int[] a, int val)
    {
        for (int i = 0, len = a.Length; i < len; i++)
            a[i] = val;
    }


    public static void fill(int[] a, int fromIndex, int toIndex, int val)
    {
        rangeCheck(a.Length, fromIndex, toIndex);
        for (int i = fromIndex; i < toIndex; i++)
            a[i] = val;
    }






    private static void rangeCheck(int arrayLength, int fromIndex, int toIndex)
    {
        if (fromIndex > toIndex)
        {
            throw new Exception("fromIndex(" + fromIndex + ") > toIndex(" + toIndex + ")");
        }
        if (fromIndex < 0)
        {
            throw new Exception("开始下标:" + fromIndex);
        }
        if (toIndex > arrayLength)
        {
            throw new Exception("结束下标：" + toIndex);
        }
    }
}

