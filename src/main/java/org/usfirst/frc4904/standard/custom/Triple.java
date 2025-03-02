package org.usfirst.frc4904.standard.custom;

public record Triple<A, B, C>(A first, B second, C third) {

    public static <A, B, C> Triple<A, B, C> of(A a, B b, C c) {
        return new Triple<>(a, b, c);
    }
}
