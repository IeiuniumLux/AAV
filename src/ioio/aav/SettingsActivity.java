package ioio.aav;

import java.util.Map;

import android.app.Activity;
import android.content.SharedPreferences;
import android.content.SharedPreferences.OnSharedPreferenceChangeListener;
import android.os.Bundle;
import android.preference.ListPreference;
import android.preference.Preference;
import android.preference.PreferenceFragment;

public class SettingsActivity extends Activity {
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);

		// Display the fragment as the main content.
		getFragmentManager().beginTransaction().replace(android.R.id.content, new SettingsFragment()).commit();
	}

	public static class SettingsFragment extends PreferenceFragment implements OnSharedPreferenceChangeListener {
		@Override
		public void onCreate(Bundle savedInstanceState) {
			super.onCreate(savedInstanceState);

			// Load the preferences from an XML resource
			addPreferencesFromResource(R.xml.settings);
		}

		@Override
		public void onResume() {
			super.onResume();
			
			// Set up initial values for all list preferences
		    Map<String, ?> sharedPreferencesMap = getPreferenceScreen().getSharedPreferences().getAll();
		    Preference pref;
		    ListPreference listPref;
		    for (Map.Entry<String, ?> entry : sharedPreferencesMap.entrySet()) {
		        pref = findPreference(entry.getKey());
		        if (pref instanceof ListPreference) {
		            listPref = (ListPreference) pref;
		            pref.setSummary(listPref.getEntry());
		        }
		    }
		    
			// Set up a listener whenever a key changes
			getPreferenceScreen().getSharedPreferences().registerOnSharedPreferenceChangeListener(this);
		}

		@Override
		public void onPause() {
			super.onPause();
			// Set up a listener whenever a key changes
			getPreferenceScreen().getSharedPreferences().unregisterOnSharedPreferenceChangeListener(this);
		}

		@Override
		public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
			Preference pref = findPreference(key);

			if (pref instanceof ListPreference) {
				ListPreference listPref = (ListPreference) pref;
				pref.setSummary(listPref.getEntry());
			}
			
			getActivity().setResult(-1);
		}
	}
}
