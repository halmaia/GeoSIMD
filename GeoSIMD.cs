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
                nuint oneVectorAwayFromEnd = bufferLength - elementCountVector512;
                
                Vector512<double> C180 = Vector512.Create(180.0d);
                Vector512<double> PI = Vector512.Create(double.Pi);

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
                nuint oneVectorAwayFromEnd = bufferLength - elementCountVector256;

                Vector256<double> C180 = Vector256.Create(180.0d);
                Vector256<double> PI = Vector256.Create(double.Pi);

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
                nuint oneVectorAwayFromEnd = bufferLength - elementCountVector128;

                Vector128<double> C180 = Vector128.Create(180.0d);
                Vector128<double> PI = Vector128.Create(double.Pi);

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
    }
}
