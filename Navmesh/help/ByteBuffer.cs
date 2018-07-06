//-------------------------------------------------------------------------------------------
//	Copyright © 2007 - 2015 Tangible Software Solutions Inc.
//	This class can be used by anyone provided that the copyright notice remains intact.
//
//	This class is used to simulate the java.nio.ByteBuffer class in C#.
//
//	Instances are only obtainable via the static 'allocate' method.
//
//	Some methods are not available:
//		All methods which create shared views of the buffer such as: array,
//		asCharBuffer, asDoubleBuffer, asFloatBuffer, asIntBuffer, asLongBuffer,
//		asReadOnlyBuffer, asShortBuffer, duplicate, slice, & wrap.
//
//		Other methods such as: mark, reset, isReadOnly, order, compareTo,
//		arrayOffset, & the limit setter method.
//-------------------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.Text;

namespace org.recast4j
{
    public class ByteBuffer
    {
        private static int MAX_BUFF_SIZE = 1024 * 1024;

        protected byte[] buffers;
        protected int mLength = 0;
        protected int mPosition = 0;

        public ByteBuffer(int bufferSize = 1024)
            : this(new byte[bufferSize], 0, 0)
        {

        }


        ByteBuffer(byte[] bytes, int mPosition, int mLength)
        {
            this.buffers = bytes;
            this.mPosition = mPosition;
            this.mLength = mLength;
        }


        public ByteBuffer slice()
        {
            return new ByteBuffer(buffers, mPosition, mLength);
        }


        public static ByteBuffer wrap(byte[] array)
        {
            return new ByteBuffer(array, 0, array.Length);
        }




        public void InitBytesArray(byte[] buff, int len)
        {
            if (len > MAX_BUFF_SIZE)
            {
                throw new Exception("InitBytesArray 初始化失败，超出字节流最大限制");
            }
            Array.Copy(buff, buffers, len);
            mLength = len;
            mPosition = 0;
        }

        public bool CreateFromSocketBuff(byte[] buff, int nSize)
        {
            if (buff == null)
            {
                return false;
            }
            this.mLength = BitConverter.ToUInt16(buff, 0);
            if ((this.mLength > MAX_BUFF_SIZE) || (this.mLength <= 0))
            {
                return false;
            }
            Array.Copy(buff, 0, this.buffers, 0, this.mLength);
            mPosition = 0;
            return true;
        }

        public bool CopyFromByteArray(ref byte[] aBuff, ref int nSize)
        {
            if (aBuff == null)
            {
                return false;
            }
            Array.Copy(this.buffers, 0, aBuff, 0, this.mLength);
            nSize = mLength;
            return true;
        }


        /// <summary>
        /// 压缩此缓冲区
        /// </summary>
        /// <returns></returns>
        public ByteBuffer Compact()
        {
            Array.Copy(this.buffers, mPosition, this.buffers, 0, Remaining);

            mPosition = Remaining;

            mLength = this.buffers.Length;

            return this;
        }



        public ByteBuffer Clear()
        {
            mPosition = 0;
            mLength = 0;

            return this;
        }




        public void WriteBool(bool value)
        {
            byte b = (byte)(value ? 0 : 1);
            buffers[mPosition] = b;
            mPosition += 1;
            mLength = mPosition;
            CheckBuffSize();
        }

        public void WriteByte(int value)
        {
            buffers[mPosition] = (byte)value;
            mPosition++;
            mLength = mPosition;
            CheckBuffSize();
        }

        public void WriteBytes(ByteBuffer bytes)
        {
            int nLen = bytes.mLength;
            Array.Copy(BitConverter.GetBytes(nLen), 0, buffers, mPosition, 2);
            mPosition += 2;
            Array.Copy(bytes.buffers, 0, buffers, mPosition, nLen);
            mPosition += nLen;
            mLength = mPosition;
            CheckBuffSize();
        }



        public void WriteBytes(byte[] bytes)
        {
            int size = buffers.Length - mPosition;

            if (bytes.Length > size)
            {
                throw new ArgumentOutOfRangeException("超出字节流最大限制。");
            }

            Array.Copy(bytes, 0, buffers, mPosition, bytes.Length);

            mPosition += bytes.Length;

            mLength = mPosition;

            CheckBuffSize();
        }


        public void WriteLong(long value)
        {
            if (mPosition + 8 > buffers.Length)
            {
                throw new ArgumentOutOfRangeException("超出字节流最大限制。");
            }
            Array.Copy(BitConverter.GetBytes(value), 0, buffers, mPosition, 8);
            mPosition += 8;
            mLength = mPosition;
            CheckBuffSize();
        }




        public void WriteDouble(double value)
        {
            Array.Copy(BitConverter.GetBytes(value), 0, buffers, mPosition, 8);
            mPosition += 8;
            mLength = mPosition;
            CheckBuffSize();
        }

        public void WriteFloat(float value)
        {
            Array.Copy(BitConverter.GetBytes(value), 0, buffers, mPosition, 4);
            mPosition += 4;
            mLength = mPosition;
            CheckBuffSize();
        }

        public void WriteInt(int value)
        {
            Array.Copy(BitConverter.GetBytes(value), 0, buffers, mPosition, 4);
            mPosition += 4;
            mLength = mPosition;
            CheckBuffSize();
        }


        public void WriteInt(int index, int value)
        {
            if (index > buffers.Length)
            {
                new ArgumentOutOfRangeException("超出字节流最大限制。");
            }

            Array.Copy(BitConverter.GetBytes(value), 0, buffers, index, 4);

        }





        public void WriteShort(int value)
        {
            Array.Copy(BitConverter.GetBytes((short)value), 0, buffers, mPosition, 2);
            mPosition += 2;
            mLength = mPosition;
            CheckBuffSize();
        }


        public void WriteChar(char c)
        {
            Array.Copy(BitConverter.GetBytes(c), 0, buffers, mPosition, 2);
            mPosition += 2;
            mLength = mPosition;
            CheckBuffSize();
        }




        public void WriteString(string value)
        {
            byte[] bytes = Encoding.UTF8.GetBytes(value);
            short strLength = (short)bytes.Length;
            Array.Copy(BitConverter.GetBytes(strLength), 0, buffers, mPosition, 2);
            mPosition += 2;
            Array.Copy(bytes, 0, buffers, mPosition, strLength);
            mPosition += strLength;
            mLength = mPosition;
            CheckBuffSize();
        }

        public bool ReadBool()
        {
            if ((mPosition + 1) > mLength)
            {
                throw new Exception("ReadBoolean读取数据失败，读取数据超出字节流范围");
            }
            bool value = buffers[mPosition] == 0 ? true : false;
            mPosition += 1;
            return value;
        }

        public byte ReadByte()
        {
            if ((mPosition + 1) > mLength)
            {
                throw new Exception("ReadByte读取数据失败，读取数据超出字节流范围");
            }
            byte value = (byte)buffers[mPosition];
            mPosition++;
            return value;
        }

        public ByteBuffer ReadBytes()
        {
            if ((mPosition + 2) > mLength)
            {
                throw new Exception("ReadBytes[0]读取数据失败，读取数据超出字节流范围");
            }
            int length = BitConverter.ToInt32(buffers, mPosition);
            mPosition += 2;
            if ((length < 0) || (length > MAX_BUFF_SIZE))
            {
                throw new Exception("ReadBytes[1]读取数据失败，读取数据超出字节流范围");
            }
            ByteBuffer values = new ByteBuffer();
            Array.Copy(buffers, mPosition, values.buffers, 0, length);
            values.mPosition = 0;
            values.mLength = length;
            mPosition += length;
            return values;
        }

        public double ReadDouble()
        {
            if ((mPosition + 8) > mLength)
            {
                throw new Exception("ReadDouble读取数据失败，读取数据超出字节流范围");
            }
            double value = BitConverter.ToDouble(buffers, mPosition);
            mPosition += 8;
            return value;
        }

        public float ReadFloat()
        {
            if ((mPosition + 4) > mLength)
            {
                throw new Exception("ReadFloat读取数据失败，读取数据超出字节流范围");
            }
            float value = BitConverter.ToSingle(buffers, mPosition);
            mPosition += 4;
            return value;
        }


        public long ReadLong()
        {
            if (mPosition + 8 > mLength)
            {
                throw new ArgumentOutOfRangeException("ReadLong读取数据失败，读取数据超出字节流范围");
            }
            long value = BitConverter.ToInt64(buffers, mPosition);
            mPosition += 8;
            return value;
        }




        public int ReadInt()
        {
            if ((mPosition + 4) > mLength)
            {
                throw new Exception("ReadInt读取数据失败，读取数据超出字节流范围");
            }
            int value = BitConverter.ToInt32(buffers, mPosition);
            mPosition += 4;
            return value;
        }

        public short ReadShort()
        {
            if ((mPosition + 2) > mLength)
            {
                throw new Exception("ReadShort读取数据失败，读取数据超出字节流范围");
            }
            short value = BitConverter.ToInt16(buffers, mPosition);
            mPosition += 2;
            return value;
        }



        public char ReadChar()
        {
            if (mPosition + 2 > mLength)
            {
                throw new ArgumentOutOfRangeException("ReadChar读取数据失败，读取数据超出字节流范围");
            }

            char c = BitConverter.ToChar(buffers, mPosition);
            mPosition += 2;
            return c;
        }



        public string ReadString()
        {
            if ((mPosition + 2) > mLength)
            {
                throw new Exception("ReadString[0]读取数据失败，读取数据超出字节流范围");
            }
            short count = BitConverter.ToInt16(buffers, mPosition);
            mPosition += 2;
            string value = Encoding.UTF8.GetString(buffers, mPosition, count);
            mPosition += count;
            return value;
        }





        private void CheckBuffSize()
        {
            if (mPosition > MAX_BUFF_SIZE)
            {
                throw new Exception("InitBytesArray初始化失败，超出字节流最大限制");
            }
        }

        public byte[] Buff
        {
            get
            {
                return buffers;
            }
        }

        public int Length
        {
            get
            {
                return mLength;
            }
            set
            {
                mLength = value;
            }
        }

        public int Postion
        {
            get
            {
                return mPosition;
            }
            set
            {
                mPosition = value;
            }
        }


        public int Remaining
        {
            get
            {
                return mLength - mPosition;
            }
        }
    }
}

