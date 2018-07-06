using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;


/// <summary>
/// sorted实现
/// </summary>
/// <typeparam name="T"></typeparam>
public class PriorityQueue<T>
{
    private object obj = new object();

    private SortedList<T, object> list;



    public PriorityQueue()
    {
        list = new SortedList<T, object>();
    }


    public PriorityQueue(IComparer<T> com)
    {
        list = new SortedList<T, object>(com);
    }



    public bool Add(T t)
    {
        if (!list.ContainsKey(t))
        {
            list.Add(t, obj);
            return true;
        }
        return false;
    }


    public void Clear()
    {
        list.Clear();
    }



    public bool Contains(T t)
    {
        return list.ContainsKey(t);
    }


    public IList<T> Iterator()
    {
        return list.Keys;
    }



    public bool Offer(T t)
    {
        if (list.ContainsKey(t))
        {
            list.Remove(t);
        }
        else
        {
            list.Add(t, obj);
        }
        //if (!list.ContainsKey(t))
        //{
        //    list.Add(t, obj);
        //    return true;
        //}
        return true;
    }


    public T Peek()
    {
        if (list.Count <= 0)
            return default(T);

        return list.Keys[0];
    }


    public T Poll()
    {
        if (list.Count <= 0)
            return default(T);

        T t = list.Keys[0];

        list.Remove(t);
        return t;
    }



    public bool Remove(T t)
    {
        return list.Remove(t);
    }


    public int Count
    {
        get
        {
            return list.Count;
        }
    }

}

