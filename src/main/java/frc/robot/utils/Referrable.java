package frc.robot.utils;

import java.util.function.Supplier;

/**
 * Class to hold a reference to an instance of T.
 * Useful for changing a member variable without referencing its referrer.
 */
public class Referrable<T> implements Supplier<T> {
	private T value;

	public Referrable(T t) {
		value = t;
	}

	public T get() {
		return value;
	}

	public void set(T t) {
		value = t;
	}
}
