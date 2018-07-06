using System;
using System.Collections.Generic;
using System.Text;


public class ByteOrder
{

    private String name;

    private ByteOrder(String name)
    {
        this.name = name;
    }



    public readonly static ByteOrder BIG_ENDIAN = new ByteOrder("BIG_ENDIAN");


    public readonly static ByteOrder LITTLE_ENDIAN = new ByteOrder("LITTLE_ENDIAN");


    public String toString()
    {
        return name;
    }

}

