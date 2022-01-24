/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.smartdashboard;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.nio.ByteBuffer;
import java.util.Set;

/**
 * The {@link SmartDashboardTab} class is the bridge between robot programs and the SmartDashboard
 * on the laptop.
 *
 * <p>When a value is put into the SmartDashboard here, it pops up on the SmartDashboard on the
 * laptop. Users can put values into and get values from the SmartDashboard.
 */
@SuppressWarnings({"PMD.GodClass", "PMD.TooManyMethods"})
public final class SmartDashboardTab {
  /** The {@link NetworkTable} used by {@link SmartDashboard}. */
  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("");

  /** The executor for listener tasks; calls listener tasks synchronously from main thread. */
  private static final ListenerExecutor listenerExecutor = new ListenerExecutor();

  static {
    HAL.report(tResourceType.kResourceType_SmartDashboard, 0);
  }

  private SmartDashboardTab() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Gets the entry for the specified key.
   *
   * @param key the key name
   * @return Network table entry.
   */
  public static NetworkTableEntry getEntry(String tabName, String key) {
    return table.getSubTable(tabName).getEntry(key);
  }

  /**
   * Checks the table and tells if it contains the specified key.
   *
   * @param key the key to search for
   * @return true if the table as a value assigned to the given key
   */
  public static boolean containsKey(String tabName, String key) {
    return table.containsKey(key);
  }

  /**
   * Get the keys stored in the SmartDashboard table of NetworkTables.
   *
   * @param types bitmask of types; 0 is treated as a "don't care".
   * @return keys currently in the table
   */
  public static Set<String> getKeys(int types) {
    return table.getKeys(types);
  }

  /**
   * Get the keys stored in the SmartDashboard table of NetworkTables.
   *
   * @return keys currently in the table.
   */
  public static Set<String> getKeys() {
    return table.getKeys();
  }

  /**
   * Makes a key's value persistent through program restarts. The key cannot be null.
   *
   * @param key the key name
   */
  public static void setPersistent(String tabName, String key) {
    getEntry(tabName, key).setPersistent();
  }

  /**
   * Stop making a key's value persistent through program restarts. The key cannot be null.
   *
   * @param key the key name
   */
  public static void clearPersistent(String tabName, String key) {
    getEntry(tabName, key).clearPersistent();
  }

  /**
   * Returns whether the value is persistent through program restarts. The key cannot be null.
   *
   * @param key the key name
   * @return True if the value is persistent.
   */
  public static boolean isPersistent(String tabName, String key) {
    return getEntry(tabName, key).isPersistent();
  }

  /**
   * Sets flags on the specified key in this table. The key can not be null.
   *
   * @param key the key name
   * @param flags the flags to set (bitmask)
   */
  public static void setFlags(String tabName, String key, int flags) {
    getEntry(tabName, key).setFlags(flags);
  }

  /**
   * Clears flags on the specified key in this table. The key can not be null.
   *
   * @param key the key name
   * @param flags the flags to clear (bitmask)
   */
  public static void clearFlags(String tabName, String key, int flags) {
    getEntry(tabName, key).clearFlags(flags);
  }

  /**
   * Returns the flags for the specified key.
   *
   * @param key the key name
   * @return the flags, or 0 if the key is not defined
   */
  public static int getFlags(String tabName, String key) {
    return getEntry(tabName, key).getFlags();
  }

  /**
   * Deletes the specified key in this table. The key can not be null.
   *
   * @param key the key name
   */
  public static void delete(String tabName, String key) {
    table.delete(key);
  }

  /**
   * Put a boolean in the table.
   *
   * @param key the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putBoolean(String tabName, String key, boolean value) {
    return getEntry(tabName, key).setBoolean(value);
  }

  /**
   * Gets the current value in the table, setting it if it does not exist.
   *
   * @param key the key
   * @param defaultValue the default value to set if key does not exist.
   * @return False if the table key exists with a different type
   */
  public static boolean setDefaultBoolean(String tabName, String key, boolean defaultValue) {
    return getEntry(tabName, key).setDefaultBoolean(defaultValue);
  }

  /**
   * Returns the boolean the key maps to. If the key does not exist or is of different type, it will
   * return the default value.
   *
   * @param key the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key or the given default value if there is no value
   *     associated with the key
   */
  public static boolean getBoolean(String tabName, String key, boolean defaultValue) {
    return getEntry(tabName, key).getBoolean(defaultValue);
  }

  /**
   * Put a number in the table.
   *
   * @param key the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putNumber(String tabName, String key, double value) {
    return getEntry(tabName, key).setDouble(value);
  }

  /**
   * Gets the current value in the table, setting it if it does not exist.
   *
   * @param key the key
   * @param defaultValue the default value to set if key does not exist.
   * @return False if the table key exists with a different type
   */
  public static boolean setDefaultNumber(String tabName, String key, double defaultValue) {
    return getEntry(tabName, key).setDefaultDouble(defaultValue);
  }

  /**
   * Returns the number the key maps to. If the key does not exist or is of different type, it will
   * return the default value.
   *
   * @param key the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key or the given default value if there is no value
   *     associated with the key
   */
  public static double getNumber(String tabName, String key, double defaultValue) {
    return getEntry(tabName, key).getDouble(defaultValue);
  }

  /**
   * Put a string in the table.
   *
   * @param key the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putString(String tabName, String key, String value) {
    return getEntry(tabName, key).setString(value);
  }

  /**
   * Gets the current value in the table, setting it if it does not exist.
   *
   * @param key the key
   * @param defaultValue the default value to set if key does not exist.
   * @return False if the table key exists with a different type
   */
  public static boolean setDefaultString(String tabName, String key, String defaultValue) {
    return getEntry(tabName, key).setDefaultString(defaultValue);
  }

  /**
   * Returns the string the key maps to. If the key does not exist or is of different type, it will
   * return the default value.
   *
   * @param key the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key or the given default value if there is no value
   *     associated with the key
   */
  public static String getString(String tabName, String key, String defaultValue) {
    return getEntry(tabName, key).getString(defaultValue);
  }

  /**
   * Put a boolean array in the table.
   *
   * @param key the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putBooleanArray(String tabName, String key, boolean[] value) {
    return getEntry(tabName, key).setBooleanArray(value);
  }

  /**
   * Put a boolean array in the table.
   *
   * @param key the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putBooleanArray(String tabName, String key, Boolean[] value) {
    return getEntry(tabName, key).setBooleanArray(value);
  }

  /**
   * Gets the current value in the table, setting it if it does not exist.
   *
   * @param key the key
   * @param defaultValue the default value to set if key does not exist.
   * @return False if the table key exists with a different type
   */
  public static boolean setDefaultBooleanArray(String tabName, String key, boolean[] defaultValue) {
    return getEntry(tabName, key).setDefaultBooleanArray(defaultValue);
  }

  /**
   * Gets the current value in the table, setting it if it does not exist.
   *
   * @param key the key
   * @param defaultValue the default value to set if key does not exist.
   * @return False if the table key exists with a different type
   */
  public static boolean setDefaultBooleanArray(String tabName, String key, Boolean[] defaultValue) {
    return getEntry(tabName, key).setDefaultBooleanArray(defaultValue);
  }

  /**
   * Returns the boolean array the key maps to. If the key does not exist or is of different type,
   * it will return the default value.
   *
   * @param key the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key or the given default value if there is no value
   *     associated with the key
   */
  public static boolean[] getBooleanArray(String tabName, String key, boolean[] defaultValue) {
    return getEntry(tabName, key).getBooleanArray(defaultValue);
  }

  /**
   * Returns the boolean array the key maps to. If the key does not exist or is of different type,
   * it will return the default value.
   *
   * @param key the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key or the given default value if there is no value
   *     associated with the key
   */
  public static Boolean[] getBooleanArray(String tabName, String key, Boolean[] defaultValue) {
    return getEntry(tabName, key).getBooleanArray(defaultValue);
  }

  /**
   * Put a number array in the table.
   *
   * @param key the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putNumberArray(String tabName, String key, double[] value) {
    return getEntry(tabName, key).setDoubleArray(value);
  }

  /**
   * Put a number array in the table.
   *
   * @param key the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putNumberArray(String tabName, String key, Double[] value) {
    return getEntry(tabName, key).setNumberArray(value);
  }

  /**
   * Gets the current value in the table, setting it if it does not exist.
   *
   * @param key the key
   * @param defaultValue the default value to set if key does not exist.
   * @return False if the table key exists with a different type
   */
  public static boolean setDefaultNumberArray(String tabName, String key, double[] defaultValue) {
    return getEntry(tabName, key).setDefaultDoubleArray(defaultValue);
  }

  /**
   * Gets the current value in the table, setting it if it does not exist.
   *
   * @param key the key
   * @param defaultValue the default value to set if key does not exist.
   * @return False if the table key exists with a different type
   */
  public static boolean setDefaultNumberArray(String tabName, String key, Double[] defaultValue) {
    return getEntry(tabName, key).setDefaultNumberArray(defaultValue);
  }

  /**
   * Returns the number array the key maps to. If the key does not exist or is of different type, it
   * will return the default value.
   *
   * @param key the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key or the given default value if there is no value
   *     associated with the key
   */
  public static double[] getNumberArray(String tabName, String key, double[] defaultValue) {
    return getEntry(tabName, key).getDoubleArray(defaultValue);
  }

  /**
   * Returns the number array the key maps to. If the key does not exist or is of different type, it
   * will return the default value.
   *
   * @param key the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key or the given default value if there is no value
   *     associated with the key
   */
  public static Double[] getNumberArray(String tabName, String key, Double[] defaultValue) {
    return getEntry(tabName, key).getDoubleArray(defaultValue);
  }

  /**
   * Put a string array in the table.
   *
   * @param key the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putStringArray(String tabName, String key, String[] value) {
    return getEntry(tabName, key).setStringArray(value);
  }

  /**
   * Gets the current value in the table, setting it if it does not exist.
   *
   * @param key the key
   * @param defaultValue the default value to set if key does not exist.
   * @return False if the table key exists with a different type
   */
  public static boolean setDefaultStringArray(String tabName, String key, String[] defaultValue) {
    return getEntry(tabName, key).setDefaultStringArray(defaultValue);
  }

  /**
   * Returns the string array the key maps to. If the key does not exist or is of different type, it
   * will return the default value.
   *
   * @param key the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key or the given default value if there is no value
   *     associated with the key
   */
  public static String[] getStringArray(String tabName, String key, String[] defaultValue) {
    return getEntry(tabName, key).getStringArray(defaultValue);
  }

  /**
   * Put a raw value (byte array) in the table.
   *
   * @param key the key to be assigned to
   * @param value the value that will be assigned
   * @return False if the table key already exists with a different type
   */
  public static boolean putRaw(String tabName, String key, byte[] value) {
    return getEntry(tabName, key).setRaw(value);
  }

  /**
   * Put a raw value (bytes from a byte buffer) in the table.
   *
   * @param key the key to be assigned to
   * @param value the value that will be assigned
   * @param len the length of the value
   * @return False if the table key already exists with a different type
   */
  public static boolean putRaw(String tabName, String key, ByteBuffer value, int len) {
    return getEntry(tabName, key).setRaw(value, len);
  }

  /**
   * Gets the current value in the table, setting it if it does not exist.
   *
   * @param key the key
   * @param defaultValue the default value to set if key does not exist.
   * @return False if the table key exists with a different type
   */
  public static boolean setDefaultRaw(String tabName, String key, byte[] defaultValue) {
    return getEntry(tabName, key).setDefaultRaw(defaultValue);
  }

  /**
   * Returns the raw value (byte array) the key maps to. If the key does not exist or is of
   * different type, it will return the default value.
   *
   * @param key the key to look up
   * @param defaultValue the value to be returned if no value is found
   * @return the value associated with the given key or the given default value if there is no value
   *     associated with the key
   */
  public static byte[] getRaw(String tabName, String key, byte[] defaultValue) {
    return getEntry(tabName, key).getRaw(defaultValue);
  }
}
