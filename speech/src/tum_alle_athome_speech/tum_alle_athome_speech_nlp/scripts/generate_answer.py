#!/usr/bin/env python
import rospy

import nltk
from nltk import tag
from nltk.util import ngrams
from collections import OrderedDict

from std_msgs.msg import String
from tum_alle_athome_speech_msgs.srv import srvTTS
from tum_alle_athome_speech_msgs.msg import msgTTSState
from tum_alle_athome_speech_msgs.srv import srvGenerateAnswer
from tum_alle_common_msgs.srv import srvAnswerReasoning
# from tum_alle_common_msgs.srv import srvExtractProperty

from subprocess import call
from subprocess import Popen
import subprocess

import answers
import ast

## Class storing basic tokenization methods
#
class ProcessText:
    ## Create a ProcessText class object.
    #
	def __init__(self):
		## Sentence processed
		self.sentence = ''
		## Answer generated
		self.answer = 'UNKNOWN'
		## List of punctuation signs
		self.punctuation = ['.', ',', ';', ':']
		## List of verb tags
		self.verbs = ['VB', 'VBD', 'VBG', 'VBN','VBZ','VBP','VERB']
		## List of verb tags
		self.nouns = ['NN', 'NNS', 'NNPS', 'NNP', 'NOUN']
		## List of ignored words
		self.ignore = ['category', 'class', 'number']
		## Language of the responses
		self.language = 'eng'
		## Extracted command
		self.command = {}
		## Person
		self.person = 'None'
		## Object
		self.object = 'None'
		## Properties
		self.properties = 'None'
		## Location
		self.location = 'None'
		## Process running
		self.proc = ''
        # self.perception_server = rospy.ServiceProxy('/recognition/extract_property', srvExtractProperty)
		self.reasoning_server = rospy.ServiceProxy('/tiago/mind/classify_query', srvAnswerReasoning)

	def to_lower(self, text):
		lowercase = text.lower()
		return lowercase

	def tokenize(self, sentence):
		tokens = nltk.word_tokenize(sentence)
		# print ("Tokens:",tokens)
		return tokens

	def tag_words(self, tokens, tag_set = None):
		tagged = nltk.pos_tag(tokens, tagset = tag_set)
		print ("Tags:", tagged)
		return tagged

	def build_tree(self, tagged):
		entities = nltk.chunk.ne_chunk(tagged)
		#print(OrderedDict(entities))
		# print ("Entities", entities)
		return entities

	def remove_trash(self, sentence):
		out_sentence = [w for w in sentence if w[1] != 'DT']
		#print out_sentence
		if out_sentence[0][0] in ['Tell','tell'] and out_sentence[1][0] == 'me':
			out_sentence = out_sentence[2:]
		#print out_sentence
		return out_sentence

	def get_human_names(self, sentence):
		sentt = nltk.ne_chunk(sentence, binary = False)
		person_list = ''
		person = []
		name = ""
		# print (sentt)
		for subtree in sentt.subtrees(filter=lambda t: t.label() == 'PERSON'):
			for leaf in subtree.leaves():
				person.append(leaf[0])
				for part in person:
					name += part + ' '
				if name[:-1] not in person_list:
					person_list = name[:-1]
				name = ''
			person = []
		# print ("Person list: ", person_list)
		return (person_list)

	def get_prp(self, pos_tag):
		operator = ''
		for (w1,t1) in pos_tag:
			if (t1.startswith('PRP')) and w1!= 'you':
				print(w1, t1)
				operator = w1
		return operator

	def get_locations(self, pos_tag, names):
		location = ''
		new_tag = self.remove_trash(pos_tag)
		for (w1,t1), (w2,t2) in nltk.bigrams(new_tag):
			if (t1.startswith('IN') and w1 != 'of' and w2 not in names):
				location = w2
			if (location == w1 and t2 in self.nouns and w2 not in names):
				location = location + ' ' + w2
			if (t1.startswith('TO') and w2 != names):
				location = w2
		return location

	def get_objects(self, pos_tag, locations = '', names = ''):
		object_list = []
		for i in range(len(pos_tag)):
			object = {}
			if (pos_tag[i][1] in self.nouns and names.find(pos_tag[i][0])==-1 and locations.find(pos_tag[i][0])==-1 and pos_tag[i][0] not in self.ignore):
				object['object'] = pos_tag[i][0]
				print (pos_tag[i][0])
				j = 1
				adj = []
				while i-j >= 0 and pos_tag[i-j][1].startswith('JJ') and pos_tag[i-j][0] != 'many':
					adj.append(pos_tag[i-j][0])
					j = j + 1
				object['properties'] = adj
				j = 1
				if i+j < len(pos_tag) and pos_tag[i+j][0] == 'of':
					while pos_tag[i+j+1][1].startswith('JJ') and j < len(pos_tag):
						j = j + 1
					object['owner'] = pos_tag[i+j+1][0]
				if (object['object']!= ''):
					object_list.append(object.copy())
		print ('Object: ', object_list)
		return object_list

	def get_properties(self, pos_tag, locations = '', names = ''):
		property = [w for w in sentence if w[1].startswith('JJ')]
		return property

	def process_intent(self, sentence, intention):
		tags = self.remove_trash(self.tag_words(self.tokenize(sentence)))
		# REASONING
		if intention == 'object_storage':
			answer_type = answers.ArenaStorage()
			command = 'The $object is stored in the $answer.'
		elif intention == 'object_stored':
			answer_type = answers.ArenaStored()
			command = 'The objects stored in the $location are $answer.'
			# REASONING
		elif intention == 'object_trivia':
			answer_type = answers.ObjectTrivia()
			command = 'The $properties $object is $answer.'
		elif intention == 'object_color':
			answer_type = answers.ObjectColor()
			command = 'The $object is $answer.'
		elif intention == 'arena_objects':
			answer_type = answers.ArenaObjects()
			command = 'There are $answer in $location.'
		elif intention == 'object_classify':
			answer_type = answers.ObjectClassify()
			command = 'The $objects belongs to $answer category.'
		elif intention == 'object_comp_cat':
			answer_type = answers.ObjectCompCat()
			command = 'The $objects $answer belong to the same category.'
			# REASONING
		elif intention == 'object_compare':
			answer_type = answers.ObjectComp()
			command = 'The $properties object is $answer.'
			# REASONING
		elif intention == 'arena_number':
			answer_type = answers.ArenaNumber()
			command = 'There are $answer $properties $object in $location.'
		elif intention == 'arena_objects':
			answer_type = answers.ArenaObjects()
			command = 'There are $answer in $location.'
			# REASONING
		# PERCEPTION
		elif intention == 'crowd_number':
			answer_type = answers.CrowdNumber()
			command = 'There are $answer $object in $location.'
			# self.proc = subprocess.Popen(["roslaunch", "tum_alle_athome_perception_objects", "pick_and_place.launch"])
		elif intention == 'crowd_age':
			answer_type = answers.CrowdAge()
			command = 'I think $properties $person $answer'
		elif intention == 'crowd_gender':
			answer_type = answers.CrowdGender()
			command = 'I think $properties $person $answer'
		print ('Pattern', command)
		# Get person
		try:
			answer_type.person_type = self.get_prp(tags)
			if answer_type.person_type == '':
				answer_type.person_type = self.get_human_names(tags)
			self.person = answer_type.person_type
		except AttributeError:
		    self.person = ''
		# Get location
		try:
			answer_type.location = self.get_locations(tags,self.person)
			self.location = answer_type.location
		except AttributeError:
			self.location = ''
		# Get objects
		temp_dict = {}
		temp_list = self.get_objects(tags,self.person,self.location)
		if temp_list != []:
			temp_dict = temp_list[0]

		try:
			if temp_dict != {}:
				answer_type.object = temp_dict['object']
				self.object = answer_type.object
			else:
				answer_type.object = 'unknown object'
				self.object = answer_type.object
		except AttributeError:
		   	self.object = ''
		try:
			if temp_dict != {}:
				answer_type.property = temp_dict['properties']
				self.properties = answer_type.property
				if answer_type.property == {}:
					self.properties = self.get_properties(tags,self.person,self.location)[0]
					answer_type.property = self.get_properties(tags,self.person,self.location)[0]
			else:
				answer_type.property = ''
				self.properties = answer_type.properties
		except AttributeError:
		   	self.properties = ''

		if self.person == 'I':
			self.person = 'you are'
		else:
			self.person = self.person + ' is'
		final = command.replace("$person", self.person)
		final = final.replace("$object", self.object)
		final = final.replace(" $properties", ' '.join(self.properties))
		if self.location != 'here' and self.location != '':
			self.location = 'the ' + self.location
		final = final.replace("$location", self.location)
		# Get an answer
		self.answer = self.get_answer(intention, answer_type)
		final = final.replace("$answer", self.answer)
		#rospy.wait_for_service('/recognition/stable_objects')
		# self.proc.kill()
		return final

	def get_answer(self, intention, request):
		# REASONING
		print (intention, request.object)
		answer = ''
		if intention == 'object_storage':
			answer = self.reasoning_server(intent = intention, properties = ['object'], values = [request.object]).answer
		elif intention == 'object_stored':
			answer = self.reasoning_server(intent = intention, properties = ['location'], values = [request.location]).answer
		elif intention == 'object_trivia':
			answer = self.reasoning_server(intent = intention, properties = ['property'], values = [request.property]).answer
		elif intention == 'object_color':
			answer = self.reasoning_server(intent = intention, properties = ['color'], values = [request.object]).answer
		elif intention == 'object_classify':
			answer = self.reasoning_server(intent = intention, properties = ['object'], values = [request.object]).answer
		elif intention == 'object_comp_cat':
			answer = self.reasoning_server(intent = intention, properties = ['objects'], values = [','.join(request.objects)]).answer
		elif intention == 'object_compare':
			answer = self.reasoning_server(intent = intention, properties = ['objects', 'property'], values = [','.join(request.objects), request.property]).answer
		elif intention == 'arena_objects':
			answer = self.reasoning_server(intent = intention, properties = ['location'], values = [request.location]).answer
		elif intention == 'arena_number':
			answer = self.reasoning_server(intent = intention, properties = ['object','location'], values = [request.object, request.location]).answer
		# PERCEPTION
		# elif intention == 'crowd_number':
		# 	answer = self.perception_server(command = '', subcommand = request.object)
		# elif intention == 'crowd_age':
		# 	answer = self.perception_server(command = 'age', subcommand = request.person)
		# elif intention == 'crowd_gender':
		# 	answer = self.perception_server(command = 'gender', subcommand = request.person)
		return str(answer)

	def basic_callback(self, msg):
		self.extract_commands(self.tag_words(self.tokenize(msg.data)))

	def question(self, req):
		return self.process_intent(req.question,req.intention)

	def question_server(self):
		s = rospy.Service('/tiago/speech/generate_answer', srvGenerateAnswer, self.question)

if __name__ == '__main__':
	rospy.init_node('speech_nlp_basic')
	rospy.loginfo("SPEECH NLP: Initializing")
	nlp_basic = ProcessText()
	rospy.Subscriber('/tiago/speech/generate_answer', String, nlp_basic.basic_callback)
	nlp_basic.question_server()
	rospy.spin()
