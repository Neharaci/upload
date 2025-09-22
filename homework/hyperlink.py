from browser_use_sdk import BrowserUseSdk

TODAY = "2025-09-22"

PEOPLE = [
    {
        "name": "Neha Rajkumar",
        "location": "Bangalore, Karnataka",
        "needs": "Information about Machine Learning",
    },
]

sdk = BrowserUseSdk(api_key="insert API key here")

for person in PEOPLE:
    result = sdk.run(
        llm_model="o3-mini",
        task=f"""
You need info about Machine Learning.

Find different sources that provide a great deal of knowledge on this topic.

  - from {person['location']}
  - on {TODAY}
  - for {person['name']}
  - who needs {person['needs']}.

Go to geeks for geeks' machine learning and gather a great deal of information on that topic and also why that website.
""",
    )