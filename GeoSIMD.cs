using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

namespace GeoSIMD
{
    public static class GeoSIMD
    {
        [SkipLocalsInit]
        public static void DegreesToRadians(Span<double> radians)
        {
            const nuint elementCountVector512 = 8u;
            const nuint elementCountVector256 = 4u;
            const nuint elementCountVector128 = 2u;

            nuint elementOffset = 0u;
            nuint bufferLength = (nuint)radians.Length;

            ref double searchSpace = ref MemoryMarshal.GetReference(radians);

            if (Vector512.IsHardwareAccelerated && bufferLength >= elementCountVector512)
            {
                Vector512<double> C180 = Vector512.Create(180.0d);
                Vector512<double> PI = Vector512.Create(double.Pi);

                nuint oneVectorAwayFromEnd = bufferLength - elementCountVector512;

                while (true)
                {
                    (PI * Vector512.LoadUnsafe(in searchSpace, elementOffset) / C180)
                                   .StoreUnsafe(ref searchSpace, elementOffset);

                    elementOffset += elementCountVector512;
                    if (elementOffset > oneVectorAwayFromEnd) break;
                }
            }
            else if (Vector256.IsHardwareAccelerated && bufferLength >= elementCountVector256)
            {
                Vector256<double> C180 = Vector256.Create(180.0d);
                Vector256<double> PI = Vector256.Create(double.Pi);

                nuint oneVectorAwayFromEnd = bufferLength - elementCountVector256;

                // According to BenchmarkDotNet's measuremnt there is no
                // measurable benefit to un-roll the while loop.

                while (true)
                {
                    (PI * Vector256.LoadUnsafe(in searchSpace, elementOffset) / C180)
                                   .StoreUnsafe(ref searchSpace, elementOffset);

                    elementOffset += elementCountVector256;

                    // If I rewrite this loop to a do ... while I get the same
                    // compillation result as now:
                    if (elementOffset > oneVectorAwayFromEnd) break;
                }
            }
            else if (Vector128.IsHardwareAccelerated && bufferLength >= elementCountVector128)
            {
                Vector128<double> C180 = Vector128.Create(180.0d);
                Vector128<double> PI = Vector128.Create(double.Pi);

                nuint oneVectorAwayFromEnd = bufferLength - elementCountVector128;

                while (true)
                {
                    (PI * Vector128.LoadUnsafe(in searchSpace, elementOffset) / C180)
                                   .StoreUnsafe(ref searchSpace, elementOffset);

                    elementOffset += elementCountVector128;
                    if (elementOffset > oneVectorAwayFromEnd) break;
                }
            }

            // Non-SIMD scalar & tail handling:
            while (elementOffset < bufferLength)
            {
                ref double scalar = ref Unsafe.Add(ref searchSpace, elementOffset++);
                scalar = double.DegreesToRadians(scalar);
            }
        }

        [SkipLocalsInit]
        public static Span<double> DegreesToRadians(ReadOnlySpan<double> radians)
        {
            const nuint elementCountVector512 = 8u;
            const nuint elementCountVector256 = 4u;
            const nuint elementCountVector128 = 2u;

            nuint elementOffset = 0u;

            ref readonly double inputSpace = ref radians.GetPinnableReference();
            nuint bufferLength = (nuint)radians.Length;
            Span<double> output = GC.AllocateUninitializedArray<double>(radians.Length);
            ref double outputSpace = ref MemoryMarshal.GetReference(output);

            // Vector512/256/128.IsHardwareAccelerated is a compilation time constant.
            if (Vector512.IsHardwareAccelerated && bufferLength >= elementCountVector512)
            {
                Vector512<double> C180 = Vector512.Create(180.0d);
                Vector512<double> PI = Vector512.Create(double.Pi);

                nuint oneVectorAwayFromEnd = bufferLength - elementCountVector512;
                while (true)
                {
                    (PI * Vector512.LoadUnsafe(in inputSpace, elementOffset) / C180)
                            .StoreUnsafe(ref outputSpace, elementOffset);
                    elementOffset += elementCountVector512;
                    if (elementOffset > oneVectorAwayFromEnd) break;
                }
            }
            else if (Vector256.IsHardwareAccelerated && bufferLength >= elementCountVector256)
            {
                Vector256<double> C180 = Vector256.Create(180.0d);
                Vector256<double> PI = Vector256.Create(double.Pi);

                nuint oneVectorAwayFromEnd = bufferLength - elementCountVector256;
                while (elementOffset <= oneVectorAwayFromEnd)
                {
                    (PI * Vector256.LoadUnsafe(in inputSpace, elementOffset) / C180)
                            .StoreUnsafe(ref outputSpace, elementOffset);
                    elementOffset += elementCountVector256;
                }
            }
            else if (Vector128.IsHardwareAccelerated && bufferLength >= elementCountVector128)
            {
                Vector128<double> C180 = Vector128.Create(180.0d);
                Vector128<double> PI = Vector128.Create(double.Pi);

                nuint oneVectorAwayFromEnd = bufferLength - elementCountVector128;
                while (true)
                {
                    (PI * Vector128.LoadUnsafe(in inputSpace, elementOffset) / C180)
                            .StoreUnsafe(ref outputSpace, elementOffset);

                    elementOffset += elementCountVector128;
                    if (elementOffset > oneVectorAwayFromEnd) break;
                }
            }

            // Non-SIMD scalar & tail handling:
            while (elementOffset < bufferLength)
            {
                ref double scalar = ref Unsafe.Add(ref outputSpace, elementOffset++);
                scalar = double.DegreesToRadians(scalar);
            }
            return output;
        }

           public static double GetArea(ReadOnlySpan<double> x, ReadOnlySpan<double> y, out bool positiveDirection)
   {
       nuint bufferLength = (nuint)x.Length;
       if (bufferLength < 4 || bufferLength != (nuint)y.Length)
           throw new IndexOutOfRangeException("Invalid vertex count.");

       const nuint elementCountVector512 = 8u;
       const nuint elementCountVector256 = 4u;
       const nuint elementCountVector128 = 2u;

       nuint elementOffset = 0u;
       double vectorSum = 0d;

       ref readonly double searchSpaceX = ref x.GetPinnableReference();
       ref readonly double searchSpaceY = ref y.GetPinnableReference();
       
       if (Vector512.IsHardwareAccelerated && bufferLength >= 2u * elementCountVector512 + 1u)
       {
           Vector512<double> sum512 = Vector512<double>.Zero;
           nuint oneVectorAwayFromEnd = bufferLength - (elementCountVector512 + 1u);
           while (true)
           {
               if (Avx512F.IsSupported)
               {
                   sum512 = Avx512F.FusedMultiplyAdd(Vector512.LoadUnsafe(in searchSpaceX, elementOffset),
                                            Vector512.LoadUnsafe(in searchSpaceY, 1 + elementOffset),
                                            Avx512F.FusedMultiplyAddNegated(Vector512.LoadUnsafe(in searchSpaceX, 1 + elementOffset),
                                                                   Vector512.LoadUnsafe(in searchSpaceY, elementOffset),
                                                                   sum512));
               }
               else
               {
                   sum512 += Vector512.LoadUnsafe(in searchSpaceX, elementOffset) *
                   Vector512.LoadUnsafe(in searchSpaceY, 1 + elementOffset) -
                   Vector512.LoadUnsafe(in searchSpaceX, 1 + elementOffset) *
                   Vector512.LoadUnsafe(in searchSpaceY, elementOffset);
               }
               elementOffset += elementCountVector512;
               if (elementOffset > oneVectorAwayFromEnd) break;
           }
           vectorSum = Vector512.Sum(.5d * sum512);
       }

       else if (Vector256.IsHardwareAccelerated && bufferLength >= 2u * elementCountVector256 + 1u)
       {
           Vector256<double> sum256 = Vector256<double>.Zero;
           nuint oneVectorAwayFromEnd = bufferLength - (elementCountVector256 + 1u);
           while (true)
           {
               if (Fma.IsSupported)
               {
                   sum256 = Fma.MultiplyAdd(Vector256.LoadUnsafe(in searchSpaceX, elementOffset),
                                            Vector256.LoadUnsafe(in searchSpaceY, 1 + elementOffset),
                                            Fma.MultiplyAddNegated(Vector256.LoadUnsafe(in searchSpaceX, 1 + elementOffset),
                                                                   Vector256.LoadUnsafe(in searchSpaceY, elementOffset),
                                                                   sum256));
               }
               else
               {
                   sum256 += Vector256.LoadUnsafe(in searchSpaceX, elementOffset) *
                   Vector256.LoadUnsafe(in searchSpaceY, 1 + elementOffset) -
                   Vector256.LoadUnsafe(in searchSpaceX, 1 + elementOffset) *
                   Vector256.LoadUnsafe(in searchSpaceY, elementOffset);
               }
               elementOffset += elementCountVector256;
               if (elementOffset > oneVectorAwayFromEnd) break;
           }
           vectorSum = Vector256.Sum(.5d * sum256);
       }
       else if (Vector128.IsHardwareAccelerated && bufferLength >= 2u * elementCountVector128 + 1)
       {
           Vector128<double> sum128 = Vector128<double>.Zero;
           nuint oneVectorAwayFromEnd = bufferLength - (elementCountVector128 + 1u);
           while (true)
           {
               if (Fma.IsSupported)
               {
                   sum128 = Fma.MultiplyAdd(Vector128.LoadUnsafe(in searchSpaceX, elementOffset),
                                            Vector128.LoadUnsafe(in searchSpaceY, 1 + elementOffset),
                                            Fma.MultiplyAddNegated(Vector128.LoadUnsafe(in searchSpaceX, 1 + elementOffset),
                                                                   Vector128.LoadUnsafe(in searchSpaceY, elementOffset),
                                                                   sum128));
               }
               else
               {
                   sum128 += Vector128.LoadUnsafe(in searchSpaceX, elementOffset) *
                   Vector128.LoadUnsafe(in searchSpaceY, 1 + elementOffset) -
                   Vector128.LoadUnsafe(in searchSpaceX, 1 + elementOffset) *
                   Vector128.LoadUnsafe(in searchSpaceY, elementOffset);
               }
               elementOffset += elementCountVector128;
               if (elementOffset > oneVectorAwayFromEnd) break;
           }
           vectorSum = Vector128.Sum(.5d * sum128);
       }

       double scalarSum = 0d;
       int index = (int)elementOffset;
       while (index < (int)(bufferLength - 1u))
       {
           scalarSum += x[index] * y[1 + index] - y[index++] * x[index];
       }

       var result = double.FusedMultiplyAdd(.5d, scalarSum, vectorSum);
       positiveDirection = double.IsPositive(result);
     
       return double.Abs(result);
   }

    }
}
