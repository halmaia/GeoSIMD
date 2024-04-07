// Written by Ákos Halmai, 2024.

using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.Arm;
using System.Runtime.Intrinsics.X86;

namespace GeoSIMD;

public static class GeoSIMD
{
    private const nuint elementCountVector512 = 8u;
    private const nuint elementCountVector256 = 4u;
    private const nuint elementCountVector128 = 2u;

    private const double s1 = 0.9999999999999375599318186670190419967059614976305799594333d;
    private const double s2 = -0.1666666666643233145818157423824897486264256985983593141745d;
    private const double s3 = 0.0083333333187655140151317116116415304845125161193612244296d;
    private const double s4 = -0.0001984126641162215009836522006852359604674565106031826592d;
    private const double s5 = 2.7556931926594908040601767274775207243915634258173167E-6d;
    private const double s6 = -2.50295188656032073490380438676828409194568629418124E-8d;
    private const double s7 = 1.540117037141464425086633145773563888660980948457E-10d;

    private const double webMercatorMaxLat = 85.051128779806592377796715521924692066982591268420688405762459391d;
    private const double webMercatorMinLat = -webMercatorMaxLat;

    private const double WGS84_SemiMajorAxis = 6378137.0d;
    private const double WGS84_HalfSemiMajorAxis = 3189068.5d;
    private const double degToRad = 0.0174532925199432957692369076848861271344287188854172545609719144d;

    [SkipLocalsInit]
    public static void DegreesToRadians(Span<double> radians)
    {
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

            // According to BenchmarkDotNet's measurement there is no
            // measurable benefit to un-roll the while loop.

            while (true)
            {
                (PI * Vector256.LoadUnsafe(in searchSpace, elementOffset) / C180)
                               .StoreUnsafe(ref searchSpace, elementOffset);

                elementOffset += elementCountVector256;

                // If I rewrite this loop to a do ... while I get the same
                // compilation result as now:
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

    [SkipLocalsInit]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double GetArea(ReadOnlySpan<double> x, ReadOnlySpan<double> y, out bool positiveDirection, bool noCheck = false)
    {
        if (!noCheck && !IsClosed(x, y))
        {
            ThrowNotClosedPolygon();

            static void ThrowNotClosedPolygon() =>
                throw new Exception("The provided polygon is not closed.");
        }

        double result = GetAreaInternal(x, y);
        positiveDirection = double.IsPositive(result);
        return double.Abs(result);
    }

    [SkipLocalsInit]
    private static double GetAreaInternal(ReadOnlySpan<double> x, ReadOnlySpan<double> y)
    {
        nuint bufferLength = (nuint)x.Length;
        if (bufferLength < 4 || bufferLength != (nuint)y.Length)
            throw new IndexOutOfRangeException("Invalid vertex count.");

        nuint elementOffset = 0u;
        double vectorSum = 0d;

        ref readonly double searchSpaceX = ref x.GetPinnableReference();
        ref readonly double searchSpaceY = ref y.GetPinnableReference();

        if (Vector512.IsHardwareAccelerated && bufferLength >= 2u * elementCountVector512 + 1u)
        {
            Vector512<double> area512 = Vector512<double>.Zero;
            nuint oneVectorAwayFromEnd = bufferLength - (elementCountVector512 + 1u);
            while (true)
            {
                if (Avx512F.IsSupported)
                {
                    area512 = Avx512F.FusedMultiplyAdd(Vector512.LoadUnsafe(in searchSpaceX, elementOffset),
                                             Vector512.LoadUnsafe(in searchSpaceY, 1 + elementOffset),
                                             Avx512F.FusedMultiplyAddNegated(Vector512.LoadUnsafe(in searchSpaceX, 1 + elementOffset),
                                                                    Vector512.LoadUnsafe(in searchSpaceY, elementOffset),
                                                                    area512));
                }
                else
                {
                    area512 += Vector512.LoadUnsafe(in searchSpaceX, elementOffset) *
                    Vector512.LoadUnsafe(in searchSpaceY, 1 + elementOffset) -
                    Vector512.LoadUnsafe(in searchSpaceX, 1 + elementOffset) *
                    Vector512.LoadUnsafe(in searchSpaceY, elementOffset);
                }
                elementOffset += elementCountVector512;
                if (elementOffset > oneVectorAwayFromEnd) break;
            }
            vectorSum = Vector512.Sum(.5d * area512);
        }

        else if (Vector256.IsHardwareAccelerated && bufferLength >= 2u * elementCountVector256 + 1u)
        {
            Vector256<double> area256 = Vector256<double>.Zero;
            nuint oneVectorAwayFromEnd = bufferLength - (elementCountVector256 + 1u);
            while (true)
            {
                if (Fma.IsSupported)
                {
                    area256 = Fma.MultiplyAdd(Vector256.LoadUnsafe(in searchSpaceX, elementOffset),
                                             Vector256.LoadUnsafe(in searchSpaceY, 1 + elementOffset),
                                             Fma.MultiplyAddNegated(Vector256.LoadUnsafe(in searchSpaceX, 1 + elementOffset),
                                                                    Vector256.LoadUnsafe(in searchSpaceY, elementOffset),
                                                                    area256));
                }
                else
                {
                    area256 += Vector256.LoadUnsafe(in searchSpaceX, elementOffset) *
                    Vector256.LoadUnsafe(in searchSpaceY, 1 + elementOffset) -
                    Vector256.LoadUnsafe(in searchSpaceX, 1 + elementOffset) *
                    Vector256.LoadUnsafe(in searchSpaceY, elementOffset);
                }
                elementOffset += elementCountVector256;
                if (elementOffset > oneVectorAwayFromEnd) break;
            }
            vectorSum = Vector256.Sum(.5d * area256);
        }
        else if (Vector128.IsHardwareAccelerated && bufferLength >= 2u * elementCountVector128 + 1)
        {
            Vector128<double> area128 = Vector128<double>.Zero;
            nuint oneVectorAwayFromEnd = bufferLength - (elementCountVector128 + 1u);
            while (true)
            {
                if (Fma.IsSupported)
                {
                    // FusedMultiplyAddNegated:
                    // "Performs a set of SIMD negated multiply-add computation on packed single-precision
                    // floating-point values using three source vectors/operands, a, b, and c.
                    // Corresponding values in two operands, a and b, are multiplied and the negated
                    // infinite precision intermediate results are added to the values in the third
                    // operand, c, after which the final results are rounded to the nearest float32 values."
                    // https://portal.nacad.ufrj.br/online/intel/compiler_c/common/core/GUID-8AD1C48E-E17D-46CC-AE7C-F7FB8EFD80DF.htm
                    area128 = Fma.MultiplyAdd(Vector128.LoadUnsafe(in searchSpaceX, elementOffset),
                                              Vector128.LoadUnsafe(in searchSpaceY, 1 + elementOffset),
                                              Fma.MultiplyAddNegated(Vector128.LoadUnsafe(in searchSpaceX, 1 + elementOffset),
                                                                     Vector128.LoadUnsafe(in searchSpaceY, elementOffset),
                                                                     area128));
                }
                // TODO: Check on real hardware!
                // It might be sub-optimal!
                else if (AdvSimd.Arm64.IsSupported)
                {
                    area128 = AdvSimd.Arm64.FusedMultiplyAdd(AdvSimd.Arm64.FusedMultiplyAdd(area128,
                                                   -Vector128.LoadUnsafe(in searchSpaceX, 1 + elementOffset),
                                                   Vector128.LoadUnsafe(in searchSpaceY, elementOffset)),
                                     Vector128.LoadUnsafe(in searchSpaceX, elementOffset),
                                     Vector128.LoadUnsafe(in searchSpaceY, 1 + elementOffset));
                }
                else
                {
                    area128 += Vector128.LoadUnsafe(in searchSpaceX, elementOffset) *
                    Vector128.LoadUnsafe(in searchSpaceY, 1 + elementOffset) -
                    Vector128.LoadUnsafe(in searchSpaceX, 1 + elementOffset) *
                    Vector128.LoadUnsafe(in searchSpaceY, elementOffset);
                }
                elementOffset += elementCountVector128;
                if (elementOffset > oneVectorAwayFromEnd) break;
            }
            vectorSum = Vector128.Sum(.5d * area128);
        }

        double scalarSum = 0d;
        int index = (int)elementOffset, oneUnitAwayFromEnd = (int)(bufferLength - 1u);
        while (index < oneUnitAwayFromEnd)
        {
            scalarSum = double.FusedMultiplyAdd(x[index], y[1 + index],
                                                double.FusedMultiplyAdd(-y[index++], x[index], scalarSum));
        }

        return double.FusedMultiplyAdd(.5d, scalarSum, vectorSum);
    }

    [SkipLocalsInit]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsClosed(ReadOnlySpan<double> x, ReadOnlySpan<double> y)
    {
        int len = x.Length;
        return len != y.Length
            ? ThrowAssymetricVertexCount()
            : len < 3 ? ThrowNotEnoughVertex()
            : (x[--len] == x[0]) && (y[len] == y[0]);

        static bool ThrowAssymetricVertexCount() =>
            throw new IndexOutOfRangeException("Asymmetric vertex count.");

        static bool ThrowNotEnoughVertex() =>
         throw new ArgumentException("There is no enough vertex to be closed.");
    }

    [SkipLocalsInit]
    // x & y have to be 'ref'!
    public static void ClosePolygon(ref Span<double> x, ref Span<double> y, bool noCheck = false)
    {
        if (noCheck || !IsClosed(x, y))
        {
            int len = x.Length;
            // If we already on the maximum size, unable to increase:
            if (len == Array.MaxLength)
                throw new IndexOutOfRangeException("The number of elements is too big, to allocate a new element.");

            Span<double> new_x = GC.AllocateUninitializedArray<double>(++len);
            x.CopyTo(new_x);
            x = new_x;

            Span<double> new_y = GC.AllocateUninitializedArray<double>(len);
            x.CopyTo(new_y);
            y = new_y;

            x[--len] = x[0];
            y[len] = y[0];
        }
    }

    public static double Min(ReadOnlySpan<double> buffer)
    {
        if (buffer.IsEmpty) throw new ArgumentException("Empty span provided.", nameof(buffer));

        nuint bufferLength = (nuint)buffer.Length;
        if (bufferLength is 1) return buffer[0];

        const nuint elementCountVector512 = 8u;
        const nuint elementCountVector256 = 4u;
        const nuint elementCountVector128 = 2u;

        ref readonly double searchSpace = ref buffer.GetPinnableReference();

        // Vector512/256/128.IsHardwareAccelerated is a compilation time constant.
        if (Vector512.IsHardwareAccelerated && bufferLength >= elementCountVector512)
        {
            Vector512<double> min512 = Vector512.LoadUnsafe(in searchSpace);
            nuint elementOffset = elementCountVector512;
            nuint oneVectorAwayFromEnd = bufferLength - elementCountVector512;

            while (elementOffset <= oneVectorAwayFromEnd)
            {
                min512 = Vector512.Min(min512, Vector512.LoadUnsafe(in searchSpace, elementOffset));
                elementOffset += elementCountVector512;
            }

            if (elementOffset < bufferLength)
            {
                min512 = Vector512.Min(min512, Vector512.LoadUnsafe(in searchSpace, oneVectorAwayFromEnd));
            }

            Vector256<double> min256 = Vector256.Min(min512.GetLower(), min512.GetUpper());
            Vector128<double> min128 = Vector128.Min(min256.GetLower(), min256.GetUpper());
            double right = min128[1], left = min128[0];
            return left < right ? left : right;
        }
        else if (Vector256.IsHardwareAccelerated && bufferLength >= elementCountVector256)
        {
            Vector256<double> min256 = Vector256.LoadUnsafe(in searchSpace);
            nuint elementOffset = elementCountVector256;
            nuint oneVectorAwayFromEnd = bufferLength - elementCountVector256;
            while (elementOffset <= oneVectorAwayFromEnd)
            {
                min256 = Vector256.Min(min256, Vector256.LoadUnsafe(in searchSpace, elementOffset));
                elementOffset += elementCountVector256;
            }

            if (elementOffset < bufferLength)
            {
                min256 = Vector256.Min(min256, Vector256.LoadUnsafe(in searchSpace, oneVectorAwayFromEnd));
            }

            Vector128<double> min128 = Vector128.Min(min256.GetLower(), min256.GetUpper());
            double right = min128[1], left = min128[0];
            return left < right ? left : right;
        }
        else if (Vector128.IsHardwareAccelerated && bufferLength >= elementCountVector128)
        {
            Vector128<double> min128 = Vector128.LoadUnsafe(in searchSpace);
            nuint elementOffset = elementCountVector128;
            nuint oneVectorAwayFromEnd = bufferLength - elementCountVector128;
            while (elementOffset <= oneVectorAwayFromEnd)
            {
                min128 = Vector128.Min(min128, Vector128.LoadUnsafe(in searchSpace, elementOffset));
                elementOffset += elementCountVector128;
            }

            if (elementOffset < bufferLength)
            {
                min128 = Vector128.Min(min128, Vector128.LoadUnsafe(in searchSpace, oneVectorAwayFromEnd));
            }

            double right = min128[1], left = min128[0];
            return left < right ? left : right;
        }

        // Non-SIMD scalar handling:
        double min = buffer[0];
        for (int i = 1; i < (int)bufferLength;)
        {
            double current = buffer[i++];
            if (current < min)
                min = current;
        }
        return min;
    }

    //public static (double minX, double minY, double maxX, double maxY) Envelope(ReadOnlySpan<double> x, ReadOnlySpan<double> y)
    //{
    //    nuint bufferLength = (nuint)x.Length;

    //    const nuint elementCountVector512 = 8u;
    //    const nuint elementCountVector256 = 4u;
    //    const nuint elementCountVector128 = 2u;

    //    // Vector512/256/128.IsHardwareAccelerated is a compilation time constant.
    //    if (Vector512.IsHardwareAccelerated && bufferLength >= elementCountVector512)
    //    {

    //        ref readonly double searchSpaceX = ref x.GetPinnableReference();
    //        ref readonly double searchSpaceY = ref y.GetPinnableReference();

    //        Vector512<double> minX512 = Vector512.LoadUnsafe(in searchSpaceX);
    //        Vector512<double> minY512 = Vector512.LoadUnsafe(in searchSpaceY);

    //        Vector512<double> maxX512 = minX512;
    //        Vector512<double> maxY512 = minY512;

    //        nuint elementOffset = elementCountVector512;
    //        nuint oneVectorAwayFromEnd = bufferLength - elementCountVector512;

    //        while (elementOffset <= oneVectorAwayFromEnd)
    //        {
    //            var currentX = Vector512.LoadUnsafe(in searchSpaceX, elementOffset);
    //            minX512 = Vector512.Min(minX512, currentX);
    //            maxX512 = Vector512.Max(maxX512, currentX);

    //            var currentY = Vector512.LoadUnsafe(in searchSpaceY, elementOffset);
    //            minY512 = Vector512.Min(minY512, currentY);
    //            maxY512 = Vector512.Max(maxY512, currentY);

    //            elementOffset += elementCountVector512;
    //        }

    //        if (elementOffset < bufferLength)
    //        {
    //            minX512 = Vector512.Min(minX512, Vector512.LoadUnsafe(in searchSpaceX, oneVectorAwayFromEnd));
    //        }

    //        Vector256<double> min256 = Vector256.Min(minX512.GetLower(), minX512.GetUpper());
    //        Vector128<double> min128 = Vector128.Min(min256.GetLower(), min256.GetUpper());
    //        double right = min128[1], left = min128[0];
    //        return left < right ? left : right;

    //    }
    //    else if (Vector256.IsHardwareAccelerated && bufferLength >= elementCountVector256)
    //    {
    //        ref readonly double searchSpace = ref x.GetPinnableReference();
    //        Vector256<double> min256 = Vector256.LoadUnsafe(in searchSpace);
    //        nuint elementOffset = elementCountVector256;
    //        nuint oneVectorAwayFromEnd = bufferLength - elementCountVector256;
    //        while (elementOffset <= oneVectorAwayFromEnd)
    //        {
    //            min256 = Vector256.Min(min256, Vector256.LoadUnsafe(in searchSpace, elementOffset));
    //            elementOffset += elementCountVector256;
    //        }

    //        if (elementOffset < bufferLength)
    //        {
    //            min256 = Vector256.Min(min256, Vector256.LoadUnsafe(in searchSpace, oneVectorAwayFromEnd));
    //        }

    //        Vector128<double> min128 = Vector128.Min(min256.GetLower(), min256.GetUpper());
    //        double right = min128[1], left = min128[0];
    //        return left < right ? left : right;
    //    }
    //    else if (Vector128.IsHardwareAccelerated && bufferLength >= elementCountVector128)
    //    {

    //        ref readonly double searchSpace = ref x.GetPinnableReference();
    //        Vector128<double> min128 = Vector128.LoadUnsafe(in searchSpace);
    //        nuint elementOffset = elementCountVector128;
    //        nuint oneVectorAwayFromEnd = bufferLength - elementCountVector128;
    //        while (elementOffset <= oneVectorAwayFromEnd)
    //        {
    //            min128 = Vector128.Min(min128, Vector128.LoadUnsafe(in searchSpace, elementOffset));
    //            elementOffset += elementCountVector128;
    //        }

    //        if (elementOffset < bufferLength)
    //        {
    //            min128 = Vector128.Min(min128, Vector128.LoadUnsafe(in searchSpace, oneVectorAwayFromEnd));
    //        }

    //        double right = min128[1], left = min128[0];
    //        return left < right ? left : right;
    //    }

    //    // Non-SIMD scalar handling:
    //    double min = x[0];
    //    for (int i = 1; i < (int)bufferLength;)
    //    {
    //        double current = x[i++];
    //        if (min > current)
    //        {
    //            min = current;
    //        }
    //    }
    //    return min;
    //}

    [SkipLocalsInit]
    public static void WGS84_WebMercator(Span<double> lon, Span<double> lat)
    {
        nuint elementOffset = 0u;
        nuint bufferLength = (nuint)lon.Length;

        ref double lon_searchSpace = ref MemoryMarshal.GetReference(lon);
        ref double lat_searchSpace = ref MemoryMarshal.GetReference(lat);

        if (Vector512.IsHardwareAccelerated && bufferLength >= elementCountVector512)
        {
            Vector512<double> v_s1 = Vector512.Create<double>(s1);
            Vector512<double> v_s2 = Vector512.Create<double>(s2);
            Vector512<double> v_s3 = Vector512.Create<double>(s3);
            Vector512<double> v_s4 = Vector512.Create<double>(s4);
            Vector512<double> v_s5 = Vector512.Create<double>(s5);
            Vector512<double> v_s6 = Vector512.Create<double>(s6);
            Vector512<double> v_s7 = Vector512.Create<double>(s7);

            Vector512<double> v_minLat = Vector512.Create<double>(webMercatorMinLat);
            Vector512<double> v_maxLat = Vector512.Create<double>(webMercatorMaxLat);
            Vector512<double> v_one = Vector512<double>.One;
            Vector512<double> v_two = Vector512.Create<double>(2.0d);

            Vector512<double> v_degToRad = Vector512.Create<double>(degToRad);

            Vector512<double> v_WGS84_HalfSemiMajorAxis = Vector512.Create<double>(WGS84_HalfSemiMajorAxis);
            Vector512<double> v_WGS84_SemiMajorAxis = Vector512.Create<double>(WGS84_SemiMajorAxis);

            nuint oneVectorAwayFromEnd = bufferLength - elementCountVector512;

            while (true)
            {
                Vector512<double> _lat = Vector512.LoadUnsafe(in lat_searchSpace, elementOffset);
                _lat = v_degToRad * Vector512.Min(v_maxLat, Vector512.Max(v_minLat, _lat));
                Vector512<double> _lat2 = _lat * _lat;

                _lat = Avx512F.IsSupported ?
                Avx512F.FusedMultiplyAdd(_lat,
                    Avx512F.FusedMultiplyAdd(_lat2,
                    Avx512F.FusedMultiplyAdd(_lat2,
                    Avx512F.FusedMultiplyAdd(_lat2,
                    Avx512F.FusedMultiplyAdd(_lat2,
                    Avx512F.FusedMultiplyAdd(_lat2,
                    Avx512F.FusedMultiplyAdd(v_s7, _lat2, v_s6), v_s5), v_s4), v_s3), v_s2), v_s1), v_one)
            :
                _lat *
                (v_s1 + _lat2 *
                (v_s2 + _lat2 *
                (v_s3 + _lat2 *
                (v_s4 + _lat2 *
                (v_s5 + _lat2 * (v_s6 + v_s7 * _lat2)))))) + v_one;

                (v_WGS84_HalfSemiMajorAxis * Vector512.Log(_lat / (v_two - _lat)))
                  .StoreUnsafe(ref lat_searchSpace, elementOffset);

                (v_WGS84_SemiMajorAxis * (v_degToRad * Vector512.LoadUnsafe(in lon_searchSpace, elementOffset)))
                    .StoreUnsafe(ref lon_searchSpace, elementOffset);

                elementOffset += elementCountVector512;
                if (elementOffset > oneVectorAwayFromEnd) break;
            }
        }
        else if (Vector256.IsHardwareAccelerated && bufferLength >= elementCountVector256)
        {
            Vector256<double> v_s1 = Vector256.Create<double>(s1);
            Vector256<double> v_s2 = Vector256.Create<double>(s2);
            Vector256<double> v_s3 = Vector256.Create<double>(s3);
            Vector256<double> v_s4 = Vector256.Create<double>(s4);
            Vector256<double> v_s5 = Vector256.Create<double>(s5);
            Vector256<double> v_s6 = Vector256.Create<double>(s6);
            Vector256<double> v_s7 = Vector256.Create<double>(s7);

            Vector256<double> v_minLat = Vector256.Create<double>(webMercatorMinLat);
            Vector256<double> v_maxLat = Vector256.Create<double>(webMercatorMaxLat);
            Vector256<double> v_one = Vector256<double>.One;
            Vector256<double> v_two = Vector256.Create<double>(2.0d);

            Vector256<double> v_degToRad = Vector256.Create<double>(degToRad);

            Vector256<double> v_WGS84_HalfSemiMajorAxis = Vector256.Create<double>(WGS84_HalfSemiMajorAxis);
            Vector256<double> v_WGS84_SemiMajorAxis = Vector256.Create<double>(WGS84_SemiMajorAxis);

            nuint oneVectorAwayFromEnd = bufferLength - elementCountVector256;

            while (true)
            {
                Vector256<double> _lat = Vector256.LoadUnsafe(in lat_searchSpace, elementOffset);
                _lat = v_degToRad * Vector256.Min(v_maxLat, Vector256.Max(v_minLat, _lat));
                Vector256<double> _lat2 = _lat * _lat;

                _lat = Fma.IsSupported ?
                       Fma.MultiplyAdd(_lat,
                       Fma.MultiplyAdd(_lat2,
                       Fma.MultiplyAdd(_lat2,
                       Fma.MultiplyAdd(_lat2,
                       Fma.MultiplyAdd(_lat2,
                       Fma.MultiplyAdd(_lat2,
                       Fma.MultiplyAdd(v_s7, _lat2, v_s6), v_s5), v_s4), v_s3), v_s2), v_s1), v_one)
                :
                       _lat *
                      (v_s1 + _lat2 *
                      (v_s2 + _lat2 *
                      (v_s3 + _lat2 *
                      (v_s4 + _lat2 *
                      (v_s5 + _lat2 * (v_s6 + v_s7 * _lat2)))))) + v_one;

                (v_WGS84_HalfSemiMajorAxis * Vector256.Log(_lat / (v_two - _lat)))
                  .StoreUnsafe(ref lat_searchSpace, elementOffset);

                (v_WGS84_SemiMajorAxis * (v_degToRad * Vector256.LoadUnsafe(in lon_searchSpace, elementOffset)))
                    .StoreUnsafe(ref lon_searchSpace, elementOffset);

                elementOffset += elementCountVector256;
                if (elementOffset > oneVectorAwayFromEnd) break;
            }
        }
        else if (Vector128.IsHardwareAccelerated && bufferLength >= elementCountVector128)
        {
            Vector128<double> v_s1 = Vector128.Create<double>(s1);
            Vector128<double> v_s2 = Vector128.Create<double>(s2);
            Vector128<double> v_s3 = Vector128.Create<double>(s3);
            Vector128<double> v_s4 = Vector128.Create<double>(s4);
            Vector128<double> v_s5 = Vector128.Create<double>(s5);
            Vector128<double> v_s6 = Vector128.Create<double>(s6);
            Vector128<double> v_s7 = Vector128.Create<double>(s7);

            Vector128<double> v_minLat = Vector128.Create<double>(webMercatorMinLat);
            Vector128<double> v_maxLat = Vector128.Create<double>(webMercatorMaxLat);
            Vector128<double> v_one = Vector128<double>.One;
            Vector128<double> v_two = Vector128.Create<double>(2.0d);

            Vector128<double> v_degToRad = Vector128.Create<double>(degToRad);

            Vector128<double> v_WGS84_HalfSemiMajorAxis = Vector128.Create<double>(WGS84_HalfSemiMajorAxis);
            Vector128<double> v_WGS84_SemiMajorAxis = Vector128.Create<double>(WGS84_SemiMajorAxis);

            nuint oneVectorAwayFromEnd = bufferLength - elementCountVector128;

            while (true)
            {
                Vector128<double> _lat = Vector128.LoadUnsafe(in lat_searchSpace, elementOffset);
                _lat = v_degToRad * Vector128.Min(v_maxLat, Vector128.Max(v_minLat, _lat));
                Vector128<double> _lat2 = _lat * _lat;

                _lat = Fma.IsSupported ?
                Fma.MultiplyAdd(_lat,
                    Fma.MultiplyAdd(_lat2,
                    Fma.MultiplyAdd(_lat2,
                    Fma.MultiplyAdd(_lat2,
                    Fma.MultiplyAdd(_lat2,
                    Fma.MultiplyAdd(_lat2,
                    Fma.MultiplyAdd(v_s7, _lat2, v_s6), v_s5), v_s4), v_s3), v_s2), v_s1), v_one)
            :
                _lat *
                (v_s1 + _lat2 *
                (v_s2 + _lat2 *
                (v_s3 + _lat2 *
                (v_s4 + _lat2 *
                (v_s5 + _lat2 * (v_s6 + v_s7 * _lat2)))))) + v_one;

                (v_WGS84_HalfSemiMajorAxis * Vector128.Log(_lat / (v_two - _lat)))
                  .StoreUnsafe(ref lat_searchSpace, elementOffset);

                (v_WGS84_SemiMajorAxis * (v_degToRad * Vector128.LoadUnsafe(in lon_searchSpace, elementOffset)))
                    .StoreUnsafe(ref lon_searchSpace, elementOffset);

                elementOffset += elementCountVector128;
                if (elementOffset > oneVectorAwayFromEnd) break;
            }
        }

        // Non-SIMD scalar & tail handling:
        while (elementOffset < bufferLength)
        {
            ref double s_lat = ref Unsafe.Add(ref lat_searchSpace, elementOffset);
            s_lat = WGS84_SemiMajorAxis * double.Atanh(
                double.Sin(double.DegreesToRadians(
                    double.Clamp(s_lat, webMercatorMinLat, webMercatorMaxLat))));

            ref double s_lon = ref Unsafe.Add(ref lon_searchSpace, elementOffset++);
            s_lon = WGS84_SemiMajorAxis * double.DegreesToRadians(s_lon);
        }
    }

    //[SkipLocalsInit]
    //public static unsafe void WGS84_WebMercator(Span<double> coords, CoordinateOrder coordinateOrder = CoordinateOrder.LatLon)
    //{
    //    nuint bufferLength = (nuint)coords.Length;
    //    if (nuint.IsOddInteger(bufferLength))
    //    {
    //        ThrowInvalidBufferLength();
    //        static void ThrowInvalidBufferLength() =>
    //            throw new ArgumentOutOfRangeException(nameof(coords), "The length of input buffer must be even and greater than or equal to 2!");
    //    }

    //    nuint elementOffset = 0u;

    //    fixed (double* baseAddr = coords)
    //    {

    //        if (Vector512.IsHardwareAccelerated && bufferLength >= 2 * elementCountVector512)
    //        {
    //            Vector512<double> v_s1 = Vector512.Create<double>(s1);
    //            Vector512<double> v_s2 = Vector512.Create<double>(s2);
    //            Vector512<double> v_s3 = Vector512.Create<double>(s3);
    //            Vector512<double> v_s4 = Vector512.Create<double>(s4);
    //            Vector512<double> v_s5 = Vector512.Create<double>(s5);
    //            Vector512<double> v_s6 = Vector512.Create<double>(s6);
    //            Vector512<double> v_s7 = Vector512.Create<double>(s7);

    //            Vector512<double> v_minLat = Vector512.Create<double>(webMercatorMinLat);
    //            Vector512<double> v_maxLat = Vector512.Create<double>(webMercatorMaxLat);
    //            Vector512<double> v_one = Vector512<double>.One;
    //            Vector512<double> v_two = Vector512.Create<double>(2.0d);

    //            Vector512<double> v_degToRad = Vector512.Create<double>(degToRad);

    //            Vector512<double> v_WGS84_HalfSemiMajorAxis = Vector512.Create<double>(WGS84_HalfSemiMajorAxis);
    //            Vector512<double> v_WGS84_SemiMajorAxis = Vector512.Create<double>(WGS84_SemiMajorAxis);

    //            Vector128<int> latIndex = coordinateOrder is CoordinateOrder.LatLon ?
    //                                      Vector128.Create(0, 2, 4, 6) :
    //                                      Vector128.Create(1, 3, 5, 7);
    //            Vector128<int> lonIndex = coordinateOrder is CoordinateOrder.LatLon ?
    //                                      Vector128.Create(1, 3, 5, 7) :
    //                                      Vector128.Create(0, 2, 4, 6);

    //            nuint oneVectorAwayFromEnd = bufferLength - 2 * elementCountVector512;

    //            while (true)
    //            {
    //                Vector512<double> _lat = Vector512.Create(Avx2.GatherVector256(baseAddr+elementOffset, latIndex, 8), Avx2.GatherVector256(baseAddr +elementOffset + elementCountVector256, latIndex, 8));
    //                Vector512<double> _lon = Vector512.Create(Avx2.GatherVector256(baseAddr+elementOffset, lonIndex, 8), Avx2.GatherVector256(baseAddr +elementOffset+ elementCountVector256, lonIndex, 8));

    //                _lat = v_degToRad * Vector512.Min(v_maxLat, Vector512.Max(v_minLat, _lat));
    //                Vector512<double> _lat2 = _lat * _lat;

    //                _lat = Avx512F.IsSupported ?
    //                Avx512F.FusedMultiplyAdd(_lat,
    //                    Avx512F.FusedMultiplyAdd(_lat2,
    //                    Avx512F.FusedMultiplyAdd(_lat2,
    //                    Avx512F.FusedMultiplyAdd(_lat2,
    //                    Avx512F.FusedMultiplyAdd(_lat2,
    //                    Avx512F.FusedMultiplyAdd(_lat2,
    //                    Avx512F.FusedMultiplyAdd(v_s7, _lat2, v_s6), v_s5), v_s4), v_s3), v_s2), v_s1), v_one)
    //            :
    //                _lat *
    //                (v_s1 + _lat2 *
    //                (v_s2 + _lat2 *
    //                (v_s3 + _lat2 *
    //                (v_s4 + _lat2 *
    //                (v_s5 + _lat2 * (v_s6 + v_s7 * _lat2)))))) + v_one;

    //                _lat = (v_WGS84_HalfSemiMajorAxis * Vector512.Log(_lat / (v_two - _lat)));
    //                _lon = (v_WGS84_SemiMajorAxis * (v_degToRad * _lon));
    //                if(coordinateOrder is CoordinateOrder.LatLon)
    //                {

    //                    Vector512.ConditionalSelect<double>(Vector512.Create(double.NaN, 0, double.NaN, 0, double.NaN, 0, double.NaN, 0), _lat, _lon)
    //                        .Store(baseAddr + elementOffset);
    //                    Vector512.ConditionalSelect<double>(Vector512.Create(double.NaN, 0, double.NaN, 0, double.NaN, 0, double.NaN, 0), _lat, _lon)
    //                    .Store(baseAddr + elementOffset + elementCountVector512);
    //                }

    //                elementOffset += 2 * elementCountVector512;
    //                if (elementOffset > oneVectorAwayFromEnd) break;
    //            }
    //        }

    //        // Non - SIMD scalar & tail handling:

    //        if (coordinateOrder is CoordinateOrder.LatLon)
    //        {
    //            while (elementOffset < bufferLength)
    //            {
    //                var address = baseAddr + elementOffset;
    //                *address = WGS84_SemiMajorAxis * double.Atanh(
    //                double.Sin(double.DegreesToRadians(
    //                    double.Clamp(*address, webMercatorMinLat, webMercatorMaxLat))));
    //                ++address;
    //                *address = WGS84_SemiMajorAxis * double.DegreesToRadians(*address);
    //                elementOffset += 2;
    //            }
    //        }
    //        else
    //        {
    //            while (elementOffset < bufferLength)
    //            {
    //                var address = baseAddr + elementOffset;
    //                *address = WGS84_SemiMajorAxis * double.DegreesToRadians(*address);

    //                ++address;
    //                *address = WGS84_SemiMajorAxis * double.Atanh(
    //                double.Sin(double.DegreesToRadians(
    //                    double.Clamp(*address, webMercatorMinLat, webMercatorMaxLat))));
    //                elementOffset += 2;
    //            }
    //        }
    //    }
    //}

}