using System.Collections.Generic;

namespace System.Collections
{

    public static class Ext
    {
        public static T[] Slice<T>(this T[] source, int fromIdx, int toIdx)
        {
            T[] ret = new T[toIdx - fromIdx + 1];
            for (int srcIdx = fromIdx, dstIdx = 0; srcIdx <= toIdx; srcIdx++)
            {
                ret[dstIdx++] = source[srcIdx];
            }

            return ret;
        }

        public static T[,] Slice<T>(this T[,] source, int fromIdxRank0, int toIdxRank0, int fromIdxRank1, int toIdxRank1)
        {
            T[,] ret = new T[toIdxRank0 - fromIdxRank0 + 1, toIdxRank1 - fromIdxRank1 + 1];

            for (int srcIdxRank0 = fromIdxRank0, dstIdxRank0 = 0; srcIdxRank0 <= toIdxRank0; srcIdxRank0++, dstIdxRank0++)
            {
                for (int srcIdxRank1 = fromIdxRank1, dstIdxRank1 = 0;
                    srcIdxRank1 <= toIdxRank1;
                    srcIdxRank1++, dstIdxRank1++)
                {
                    ret[dstIdxRank0, dstIdxRank1] = source[srcIdxRank0, srcIdxRank1];
                }
            }

            return ret;
        }
    }
}