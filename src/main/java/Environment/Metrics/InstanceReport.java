package Environment.Metrics;

import java.io.IOException;
import java.util.*;

/**
 * This class is used to collect metrics about a single run of a single instance. It is strongly coupled with the
 * {@link S_Metrics} class.
 * Will only allow one occurrence of a field name. Meaning there can'y be a String field and an Integer field with the
 * same name.
 */
public class InstanceReport {

    //max cell size of excel, plus room for wrapping with "" plus some safety
    private static final int MAX_STRING_SIZE = 32767 - 12;
    public static final String EXTENSION_STRING = " - Extended ";

    private Map<String, String> stringFields = new HashMap<String, String>(8);
    private Map<String, Integer> integerFields = new HashMap<String, Integer>(8);
    private Map<String, Float> floatFields = new HashMap<String, Float>(4);
    private boolean isCommited = false;

    /**
     * Contains constants representing standard names for fields. It is optional to use these names, and any other field
     * names would also be accepted by this class.
     */
    public static class StandardFields{
        public final static String experimentName = "Experiment Name";
        public final static String instanceName = "Instance Name";
        public final static String mapName = "Map Name";
        public final static String numAgents = "# Agents";
        public final static String obstacleRate = "% Obstacles";
        public final static String solver = "Solver";
        public final static String expandedNodesLowLevel = "Expanded Nodes (Low Level)";
        public final static String expandedNodes = "Expanded Nodes (High Level)";
        public final static String generatedNodesLowLevel = "Generated Nodes(Low Level)";
        public final static String generatedNodes = "Generated Nodes  (High Level)";
        public final static String startDateTime = "Start Date";
        public final static String endDateTime = "End Date";
        public final static String elapsedTimeMS = "Elapsed Time (ms)";
        public final static String totalLowLevelTimeMS = "Total Low Level Time (ms)";
        public final static String timeoutThresholdMS = "Timeout Threshold";
        public final static String solved = "Solved";
        public final static String valid = "Valid";
        public final static String solutionCost = "Solution Cost";
        public final static String totalReroutesCost = "Total Cost of Reroutes";
        public final static String COR = "cost of reroute";
        public final static String solutionCostFunction = "Cost Function";
        public final static String solution = "Solution";
        public final static String agentSelection = "Agent Selection Policy";
        public final static String arrivalDistribution = "Arrival Distribution";
        public final static String arrivalRate = "Arrival Rate";
        public final static String skipped = "Skipped";
        public final static String numReroutes = "# Reroutes";
        public final static String capacityBreached = "Capacity Breached";
        public final static String throughput = "Throughput";
    }

    /**
     * Stores the value to the given field. If the field (fieldName) already exists, but with a different type,
     * does nothing.
     * @param fieldName the name of the field. If intended for CSV output, can't contain ','. Can't contain {@link #EXTENSION_STRING}. @NotNull
     * @param fieldValue the value to associate with the field. If intended for CSV output, can't contain ','. @NotNull
     * @return the old value of the field (first part if the field was too large and split into multiple fields). Returns
     * null if it didn't exist. also returns null if the field (fieldName) already exists, but with a different type, or
     * it the fieldName contains {@link #EXTENSION_STRING}.
     */
    public String putStringValue(String fieldName, String fieldValue){
        if(!canPutToMap(fieldName, stringFields)) {return null;}
        if(fieldName.contains(EXTENSION_STRING)) {return null;}
        removeExtensions(fieldName);
        if(fieldValue.length() > MAX_STRING_SIZE){ //split if too large, to display properly in excel
            //nicetohave -  this wasn't displaying properly so I just made it keep only the first part of the solution. May fix to keep entire solution
//            return putStringInParts(fieldName, fieldValue);
            return this.stringFields.put(fieldName, fieldValue.substring(0, MAX_STRING_SIZE - 9) + "...");
        }
        else{
            //wrap with " for csv compliance
            return this.stringFields.put(fieldName, fieldValue);
        }
    }

    /**
     * When a String field is too long, it is extended. This removes those extensions if they exist. It is necessary to
     * do so before replacing the value of the field.
     * @param fieldName
     */
    private void removeExtensions(String fieldName) {
        int extensionIndex = 1;
        boolean done = false;
        while (!done){
            String oldValue = stringFields.remove(extensionFieldName(fieldName, extensionIndex));
            done = oldValue == null;
            extensionIndex++;
        }
    }

    private String putStringInParts(String fieldName, String fieldValue) {
        int extensionIndex = 0;
        int i = 0;
        String result = null;
        for (; i < fieldValue.length(); i+= MAX_STRING_SIZE ) {
            //create substring
            int endSubstringIndex = Math.min(i + MAX_STRING_SIZE, fieldValue.length());
            String wrappedSubstring = fieldValue.substring(i, endSubstringIndex);

            if(extensionIndex == 0){ // not an extension yet, use original field name.
                result = stringFields.put(fieldName, wrappedSubstring);
            }
            else { //is an extension
                stringFields.put(extensionFieldName(fieldName, extensionIndex), wrappedSubstring);
            }
            extensionIndex++;
        }
        return result;
    }

    private String extensionFieldName(String originalFieldName, int extensionIndex){
        return originalFieldName + EXTENSION_STRING + extensionIndex;
    }

    /**
     * @param fieldName the name of the field. @NotNull
     * @return the value of the field, or null if it doesn't exist for this type.
     */
    public String getStringValue(String fieldName){
        return this.stringFields.get(fieldName);
    }

    /**
     * Stores the value to the given field. If the field (fieldName) already exists, but with a different type,
     * does nothing.
     * @param fieldName the name of the field. If intended for CSV output, can't contain ','. @NotNull
     * @param fieldValue the value to associate with the field. @NotNull
     * @return the old value of the field, or null if it didn't exist. also returns null if the field (fieldName) already exists, but with a different type.
     */
    public Integer putIntegerValue(String fieldName, int fieldValue){
        if(!canPutToMap(fieldName, integerFields)) {return null;}
        return this.integerFields.put(fieldName, fieldValue);
    }


    /**
     * @param fieldName the name of the field. @NotNull
     * @return the value of the field, or null if it doesn't exist for this type.
     */
    public Integer getIntegerValue(String fieldName){
        return this.integerFields.get(fieldName);
    }

    /**
     * Stores the value to the given field. If the field (fieldName) already exists, but with a different type,
     * does nothing.
     * @param fieldName the name of the field. If intended for CSV output, can't contain ','. @NotNull
     * @param fieldValue the value to associate with the field. @NotNull
     * @return the old value of the field, or null if it didn't exist. also returns null if the field (fieldName) already exists, but with a different type.
     */
    public Float putFloatValue(String fieldName, float fieldValue){
        if(!canPutToMap(fieldName, floatFields)) {return null;}
        return this.floatFields.put(fieldName, fieldValue);
    }


    /**
     * @param fieldName the name of the field. @NotNull
     * @return the value of the field, or null if it doesn't exist for this type.
     */
    public Float getFloatValue(String fieldName){
        return this.floatFields.get(fieldName);
    }

    /**
     * Adds the value to the currently held value. if the field already exists with another associated type, does
     * nothing. If the field doesn't exist yet, saves it with addToValue as its value.
     * @param fieldName the field to perform an addition on.
     * @param addToValue the value to add (addition) to the current value of the field.
     * @return the new value for the field, or null if the field is associated with a different type.
     */
    public Integer integerAddition(String fieldName, int addToValue){
        if(!canPutToMap(fieldName, integerFields)) {return null;}
        Integer original = this.integerFields.get(fieldName);
        Integer newValue = (original == null ? 0 : original) + addToValue;
        return this.integerFields.put(fieldName, newValue);
    }

    /**
     * Multiplies the value with the currently held value. if the field already exists with another associated type, does
     * nothing. If the field doesn't exist yet, saves it with 0 as its value.
     * @param fieldName the field to perform an multiplication on.
     * @param multiplyValueWith the value by which to multiply the current value of the field.
     * @return the new value for the field, or null if the field is associated with a different type.
     */
    public Integer integerMultiplication(String fieldName, int multiplyValueWith){
        if(!canPutToMap(fieldName, integerFields)) {return null;}
        Integer original = this.integerFields.get(fieldName);
        Integer newValue = (original == null ? 0 : original) * multiplyValueWith;
        return this.integerFields.put(fieldName, newValue);
    }

    /**
     * Adds the value to the currently held value. if the field already exists with another associated type, does
     * nothing. If the field doesn't exist yet, saves it with addToValue as its value.
     * @param fieldName the field to perform an addition on.
     * @param addToValue the value to add (addition) to the current value of the field.
     * @return the new value for the field, or null if the field is associated with a different type.
     */
    public Float floatAddition(String fieldName, float addToValue){
        if(!canPutToMap(fieldName, floatFields)) {return null;}
        Float original = this.floatFields.get(fieldName);
        Float newValue = (original == null ? 0 : original) + addToValue;
        return this.floatFields.put(fieldName, newValue);
    }

    /**
     * Multiplies the value with the currently held value. if the field already exists with another associated type, does
     * nothing. If the field doesn't exist yet, saves it with 0 as its value.
     * @param fieldName the field to perform an multiplication on.
     * @param multiplyValueWith the value by which to multiply the current value of the field.
     * @return the new value for the field, or null if the field is associated with a different type.
     */
    public Float floatMultiplication(String fieldName, float multiplyValueWith){
        if(!canPutToMap(fieldName, floatFields)) {return null;}
        Float original = this.floatFields.get(fieldName);
        Float newValue = (original == null ? 0 : original) * multiplyValueWith;
        return this.floatFields.put(fieldName, newValue);
    }

    /**
     * @param fieldName the field to check.
     * @return true if the report has a value for the given field name.
     */
    public boolean hasField(String fieldName){
        return stringFields.containsKey(fieldName) || integerFields.containsKey(fieldName)
                || floatFields.containsKey(fieldName);
    }

    /**
     * @return a {@link Set} of the field names for which values are currently held.
     */
    public Set<String> getAllFields() {
        HashSet<String> allFields = new HashSet<String>(stringFields.keySet());
        allFields.addAll(integerFields.keySet());
        allFields.addAll(floatFields.keySet());
        return allFields;
    }

    /**
     * a field can be put to a map if it already contains it, or if no other map contains it.
     * @param fieldName
     * @param map the map that we may want to add/update the field to/in.
     * @return true if the map contains, ot may can contain the field name (key).
     */
    private boolean canPutToMap(String fieldName, Map map){
        if(map.containsKey(fieldName)){
            return true;
        }
        else{
            return !hasField(fieldName);
        }
    }

    /**
     * returns a value associated with the field, if one exists, else returns null.
     * @param fieldName the name of the field.
     * @return a value associated with the field, if one exists, else returns null.
     */
    String getValue(String fieldName){
        Object value = this.getStringValue(fieldName);
        if(value == null){
            value = this.getIntegerValue(fieldName);
            if(value == null){
                value = this.getFloatValue((fieldName));
            }
        }
        return value != null ? value.toString() : null;
    }

    @Override
    public String toString() {
        return "InstanceReport{" +
                "stringFields=" + stringFields +
                ", integerFields=" + integerFields +
                ", floatFields=" + floatFields +
                '}';
    }

    /**
     * Commits the report, signaling to the {@link S_Metrics} class that the report is final and that it
     * can output the report to its output streams. An instance of this class can only be committed one. Repeated calls
     * to this method will have no effect.
     * Note that to use CSV output format (the default), you will have to provide {@link S_Metrics} with a header before
     * calling this method, else there will be no output.
     * @return true if this is the first call to this method on this instance, else false.
     */
    public boolean commit() throws IOException {
        if(isCommited){
            return false;
        }
        else{
            S_Metrics.commit(this);
            isCommited = true;
            return true;
        }
    }

}
