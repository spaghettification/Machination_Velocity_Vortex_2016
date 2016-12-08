



public class AndroidIO {
SharePreferences pref = getApplicationContext().getSharedPreferences("MyPref",MODE_PUBLIC);
Editor editor = pref.edit();

public boolean IsFileWritable(File file){return false;}
public boolean IsFileReadable(File file){return false;}
public void WriteDoubleToFile(File file,String name, double Data){
editor.putFloat(name,(float)Data);
editor.apply();
}
public double ReadDoubleFromFile(File file, String name){
float data=pref.getFloat(name,null);
return Data; }
}
